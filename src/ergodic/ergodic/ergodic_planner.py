import time
import math

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg._point import Point
from rclpy.node import Node
from std_srvs.srv import SetBool
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from scipy.stats import multivariate_normal as mvn

from data_interfaces.msg import Robot
from ergodic.sandbox import iLQR_ergodic_pointmass
import threading
from enum import auto, Enum
import csv

np.set_printoptions(precision=4)
rng = np.random.default_rng(1)

# Define the target distribution
mean1 = np.array([0.45, 0.00])
cov1 = np.array([[0.01, 0.0], [0.0, 0.01]])
w1 = 1.0


def pdf(x):
    """Return the probability density of a single Gaussian distribution evaluated at x."""
    return w1 * mvn.pdf(x, mean1, cov1)


class State(Enum):
    """Enum to represent the current state of the planner."""

    IDLE = auto()
    PLANNING = auto()
    RECORDING = auto()
    READY = auto()
    MOVING = auto()
    SHUTDOWN = auto()


class ErgodicPlanner(Node):
    """
    ROS2 node that performs ergodic trajectory planning for a point mass robot.

    Subscribes to robot state, plans trajectories using iLQR, and publishes goals.
    """

    def __init__(self):
        """Initialize the ErgodicPlanner node."""
        super().__init__('ergodic_planner')
        self.robot_state_subscription = self.create_subscription(Robot, 'robot_data', self.listener_callback, 10)
        self.ergodic_goal_publisher = self.create_publisher(Point, 'ergodic_goal', 10)
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.ergodic_goal_toggle = self.create_client(SetBool, 'toggle_goal_usage', callback_group=self.service_group)
        self.sending_goal = False
        self.robot_state_subscription  # prevent unused variable warning
        self.timer = self.create_timer(0.1, self.timer_callback)

        # manage user input
        self.running = True
        self.input_thread = threading.Thread(target=self.user_input_loop, daemon=True)

        self.recordBuffer = []
        self.currentTrajectories = []
        self.currentTraj = []
        self.pdf_recon = None

        self.state = State.IDLE
        self.position = None
        self.goal = np.array([0.35, 0.0])
        self.staticHeight = 0.590467
        self.goalIndex = 0

        # Define a 1-by-1 2D search space
        dim_root = np.array([0.30, -0.25])
        self.dim_root = dim_root
        L_list = np.array([0.45, 0.5])
        self.L_list = L_list

        # Discretize the search space into 100-by-100 mesh grids
        grids_x, grids_y = np.meshgrid(
            np.linspace(dim_root[0], dim_root[0] + L_list[0], 100),
            np.linspace(dim_root[1], dim_root[1] + L_list[1], 100),
        )
        self.grids_x = grids_x
        self.grids_y = grids_y
        grids = np.array([grids_x.ravel(), grids_y.ravel()]).T
        dx = 1.0 / 99
        dy = 1.0 / 99
        self.dx = dx
        self.dy = dy
        self.grids = grids

        # Frequency vectors
        num_k_per_dim = 10
        ks_dim1, ks_dim2 = np.meshgrid(np.arange(num_k_per_dim), np.arange(num_k_per_dim))
        ks = np.array([ks_dim1.ravel(), ks_dim2.ravel()]).T
        self.ks = ks

        # Lambda and h_k coefficients
        self.lamk_list = np.power(1.0 + np.linalg.norm(ks, axis=1), -3 / 2.0)
        hk_list = np.zeros(ks.shape[0])
        self.hk_list = hk_list
        for i, k_vec in enumerate(ks):
            fk_vals = np.prod(np.cos(np.pi * k_vec / L_list * grids), axis=1)
            hk = np.sqrt(np.sum(np.square(fk_vals)) * dx * dy)
            hk_list[i] = hk

        plt.ion()
        self.fig, self.axes = plt.subplots(1, 2, dpi=70, figsize=(25, 20), tight_layout=True)
        self.input_thread.start()

    def phiKFromTraj(self, x_traj):
        """Produce the phik coefficients form a given x-y trajectory."""
        phik_list = np.zeros(self.ks.shape[0])

        for i, (k_vec, hk) in enumerate(zip(self.ks, self.hk_list)):
            fk_vals = np.prod(np.cos(np.pi * k_vec / self.L_list * x_traj), axis=1)
            fk_vals /= hk
            phik = np.mean(fk_vals)  # Time average
            phik_list[i] = phik

        return phik_list

    def plan(self):
        """
        Plan an ergodic trajectory using iterative LQR (iLQR).

        The resulting trajectory and loss history are stored.
        """
        # Coefficients for target distribution
        # phik_list = np.zeros(ks.shape[0])
        # pdf_vals = pdf(grids)
        # self.pdf_vals = pdf_vals
        # for i, (k_vec, hk) in enumerate(zip(ks, hk_list)):
        #     fk_vals = np.prod(np.cos(np.pi * k_vec / L_list * grids), axis=1)
        #     fk_vals /= hk
        #     phik = np.sum(fk_vals * pdf_vals) * dx * dy
        #     phik_list[i] = phik

        phik_list = sum(self.phiKFromTraj(traj - self.dim_root) for traj in self.currentTrajectories)

        pdf_recon = np.zeros(self.grids.shape[0])
        for i, (phik, k_vec) in enumerate(zip(phik_list, self.ks)):
            fk_vals = np.prod(np.cos(np.pi * k_vec / self.L_list * (self.grids - self.dim_root)), axis=1)
            hk = np.sqrt(np.sum(np.square(fk_vals)) * self.dx * self.dy)
            fk_vals /= hk

            pdf_recon += phik * fk_vals

        pdf_recon = np.maximum(pdf_recon, 0)
        pdf_recon /= np.sum(pdf_recon) * self.dx * self.dy
        self.pdf_recon = pdf_recon

        # Trajectory optimizer setup
        dt = 0.1
        self.dt = dt
        tsteps = 100
        self.tsteps = tsteps
        R = np.diag([0.0001, 0.0001])
        Q_z = np.diag([0.1, 0.1])
        R_v = np.diag([0.01, 0.01])

        self.trajopt_ergodic_pointmass = iLQR_ergodic_pointmass(
            dt,
            tsteps,
            x_dim=2,
            u_dim=2,
            Q_z=Q_z,
            R_v=R_v,
            R=R,
            ks=self.ks,
            L_list=self.L_list,
            lamk_list=self.lamk_list,
            hk_list=self.hk_list,
            phik_list=phik_list,
        )

        temp_x_traj = np.array(
            [
                np.linspace(0.0, 0.15, tsteps + 1) * np.cos(np.linspace(0.0, 2 * np.pi, tsteps + 1)),
                np.linspace(0.0, 0.15, tsteps + 1) * np.sin(np.linspace(0.0, 2 * np.pi, tsteps + 1)),
            ]
        ).T
        self.init_u_traj = (temp_x_traj[1:, :] - temp_x_traj[:-1, :]) / dt

        u_traj = self.init_u_traj.copy()
        loss_list = []
        x0 = self.x0 - self.dim_root
        # startTime = time.time()
        while len(loss_list) < 2 or (loss_list[-2] - loss_list[-1]) > 0.001:
            x_traj = self.trajopt_ergodic_pointmass.traj_sim(x0, u_traj)
            v_traj = self.trajopt_ergodic_pointmass.get_descent(x0, u_traj)

            loss_val = self.trajopt_ergodic_pointmass.loss(x_traj, u_traj)
            loss_list.append(loss_val)

            step = 0.002
            alpha = 0.5
            for _ in range(3):
                temp_u_traj = u_traj + step * v_traj
                temp_x_traj = self.trajopt_ergodic_pointmass.traj_sim(x0, temp_u_traj)
                temp_loss_val = self.trajopt_ergodic_pointmass.loss(temp_x_traj, temp_u_traj)
                if temp_loss_val < loss_val:
                    break
                else:
                    step *= alpha
            u_traj += step * v_traj
            self.x_traj = x_traj
            self.u_traj = u_traj

        self.loss_list = loss_list
        # print(f'Planning complete in {time.time() - startTime} seconds. {len(loss_list)} iterations.')

    def draw(self):
        """Update the plots with the current trajectory, control inputs, and objective history."""
        axes = self.axes
        dim_root = self.dim_root
        L_list = self.L_list
        x_traj = self.x_traj
        # tsteps = self.tsteps
        x0 = self.x0
        # u_traj = self.u_traj
        # dt = self.dt
        # loss_list = self.loss_list
        pdf_vals = self.pdf_recon
        grids_x = self.grids_x
        grids_y = self.grids_y

        ax1 = axes[0]
        ax1.cla()
        ax1.set_aspect('equal', adjustable='box')
        ax1.set_xlim(dim_root[0], dim_root[0] + L_list[0])
        ax1.set_ylim(dim_root[1], dim_root[1] + L_list[1])
        ax1.set_title('Map')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.contourf(grids_x, grids_y, pdf_vals.reshape(grids_x.shape), cmap='Reds')
        for traj in self.currentTrajectories:
            trajX, trajY = zip(*traj)
            ax1.plot(trajX, trajY, linestyle='-', color='green', linewidth=2, alpha=1.0)
        ax1.plot(
            x_traj[:, 0] + dim_root[0],
            x_traj[:, 1] + dim_root[1],
            linestyle='-',
            marker='o',
            color='k',
            linewidth=2,
            alpha=1.0,
        )
        ax1.plot(x0[0], x0[1], linestyle='', marker='o', markersize=15, color='C0', alpha=1.0, label='Robot')
        ax1.legend(loc=1)

        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def listener_callback(self, msg):
        """
        Receive the robot's current position.

        Updates initial state x0 for planning.
        """
        self.position = msg.position.position

        # update state of x0 used in planner
        self.x0 = np.array([self.position.x, self.position.y])

        # if we are recording and if this is a new spot, track it in buffer
        if self.state == State.RECORDING:
            if len(self.recordBuffer) < 1 or np.any(
                np.abs(self.recordBuffer[-1] - [self.position.x, self.position.y]) > 0.001
            ):
                self.recordBuffer.append(np.array([self.position.x, self.position.y]))

    def timer_callback(self):
        """
        Periodic timer callback that manages the planner's state machine.

        Handles planning, visualization, and goal publishing.
        """
        if self.position is not None:
            # idle is resting state, indicates no path is loaded
            if self.state == State.PLANNING:
                if len(self.currentTrajectories) > 0:
                    self.plan()
                    self.state = State.READY

            # if a path is loaded, we can draw
            if self.state == State.READY or self.state == State.MOVING:
                self.draw()

            # if we are moving, start sending waypoints to the controller
            if self.state == State.MOVING:
                if math.dist(self.x0, self.goal) < 0.02:
                    self.goalIndex += 1
                self.goal = self.x_traj[self.goalIndex] + self.dim_root

                msg = Point()
                msg.x = self.goal[0]
                msg.y = self.goal[1]
                msg.z = self.staticHeight
                self.ergodic_goal_publisher.publish(msg)

                # tell the controller to begin listening to waypoints
                if not self.sending_goal:
                    req = SetBool.Request()
                    req.data = True
                    self.future = self.ergodic_goal_toggle.call_async(req)
                    rclpy.spin_until_future_complete(self, self.future)
                    self.sending_goal = True

                # if we have reached the end of the planned trajectory, disable controller waypoints and return to idle
                if self.goalIndex > len(self.x_traj) - 1:
                    self.state = State.IDLE
                    req = SetBool.Request()
                    req.data = False
                    self.future = self.ergodic_goal_toggle.call_async(req)
                    rclpy.spin_until_future_complete(self, self.future)
                    self.sending_goal = False
                    self.goalIndex = 0

    def user_input_loop(self):
        """Thread loop that listens for user input from the terminal."""
        print('Available states: shutdown, record, load, plan, moving')
        while not self.state == State.SHUTDOWN:
            user_input = input('Enter state: ').strip().lower()

            # kill the app
            if user_input == 'shutdown':
                self.state = State.SHUTDOWN
                rclpy.shutdown()

            # begin sending waypoints to follow a planned trajectory
            if user_input == 'moving':
                self.state = State.MOVING

            # record a new trajectory from the arm and write to a file
            if user_input == 'record':
                self.state = State.RECORDING
                label = input(
                    'Recording. Type "name" and either "good" or "bad" to complete the capture and provide a label: '
                )

                self.currentTrajectories.append(self.recordBuffer)
                label = label.split()
                outputFile = 'saved_data/trajectories/' + label[0] + '-' + label[1] + '.csv'
                with open(outputFile, 'w', newline='') as csvFile:
                    csv_writer = csv.writer(csvFile)
                    csv_writer.writerows(self.recordBuffer)
                self.recordBuffer = []
                self.state = State.IDLE

            # load a trajectory from a file and save the trajectory
            if user_input == 'load':
                fileName = input('Provide the name of the csv file to be loaded: ')
                points = []
                with open('saved_data/trajectories/' + fileName, 'r', newline='') as csvfile:
                    csv_reader = csv.reader(csvfile)
                    for row in csv_reader:
                        points.append(np.array(row, dtype=np.float64))
                self.currentTrajectories.append(points)
                print('Trajectory loaded')

            if user_input == 'plan':
                self.state = State.PLANNING


def main(args=None):
    """Entry point for running the ErgodicPlanner node."""
    rclpy.init(args=args)
    ergodic_planner = ErgodicPlanner()
    rclpy.spin(ergodic_planner)


if __name__ == '__main__':
    main()
