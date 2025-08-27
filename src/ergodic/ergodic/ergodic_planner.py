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
import colorsys

import matplotlib.cm as cm
import matplotlib.colors as mcolors

np.set_printoptions(precision=4)
rng = np.random.default_rng(1)

cmap = cm.coolwarm
norm = mcolors.Normalize(vmin=-1, vmax=1)


def normalize_labels(labelList):
    negatives = [val for val in labelList if val < 0]

    if negatives:
        min_val = min(negatives)  # Most negative value (e.g., -7.3)
        normalizedLabels = [val if val == 1 else (val / abs(min_val)) for val in labelList]
    else:
        normalizedLabels = labelList[:]

    return normalizedLabels


def bounded_spiral(start_xy, bounds, tsteps=100, max_radius=0.15, num_loops=2):
    """
    Generate a spiral trajectory starting from a given point and staying within bounds.

    Parameters:
        start_xy (tuple): Starting point (x, y).
        bounds (tuple): ((xmin, xmax), (ymin, ymax))
        tsteps (int): Number of steps.
        max_radius (float): Maximum spiral radius.
        num_loops (float): How many loops in the spiral.

    Returns:
        np.ndarray of shape (tsteps+1, 2)
    """
    xmax = bounds[0]
    ymax = bounds[1]

    # Define spiral in polar coordinates
    r = np.linspace(0.0, max_radius, tsteps + 1)
    theta = np.linspace(0.0, num_loops * 2 * np.pi, tsteps + 1)

    dx = r * np.cos(theta)
    dy = r * np.sin(theta)

    spiral = np.stack([dx, dy], axis=1)
    traj = spiral + np.array(start_xy)

    # Clip trajectory to stay within bounds
    traj[:, 0] = np.clip(traj[:, 0], 0.01, xmax - 0.01)
    traj[:, 1] = np.clip(traj[:, 1], 0.01, ymax - 0.01)

    return traj


class State(Enum):
    """Enum to represent the current state of the planner."""

    IDLE = auto()
    PLANNING = auto()
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
        self.recordLabels = []
        self.currentTrajectories = []
        self.pdf_recon = None

        self.state = State.IDLE
        self.isRecording = False
        self.position = None
        self.goal = np.array([0.35, 0.0])
        self.x0 = np.array([0.35, 0.0])
        self.staticHeight = 0.590467
        self.goalIndex = 0

        # Define a 1-by-1 2D search space
        dim_root = np.array([0.30, -0.25])
        self.dim_root = dim_root
        L_list = np.array([0.40, 0.5])
        self.L_list = L_list
        self.dt = 0.05
        self.tsteps = 100

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
        num_k_per_dim = 15
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
        self.fig = plt.figure(figsize=(20, 20), constrained_layout=True)
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.colorbar = None
        self.input_thread.start()

    def phiKFromTraj(self, x_traj, labels):
        """Produce the phik coefficients form a given x-y trajectory."""
        phik_list = np.zeros(self.ks.shape[0])

        for i, (k_vec, hk) in enumerate(zip(self.ks, self.hk_list)):
            fk_vals = np.prod(np.cos(np.pi * k_vec / self.L_list * x_traj), axis=1)
            fk_vals /= hk

            # labels indicate whether the point in the trajectory is constructive or reductive
            fk_vals = fk_vals * labels

            phik = np.mean(fk_vals)  # Time average
            phik_list[i] = phik

        return phik_list

    def plan(self):
        """
        Plan an ergodic trajectory using iterative LQR (iLQR).

        The resulting trajectory and loss history are stored.
        """
        # good trajectories represent posotive weighting while bad trajectories represent negative
        totalPoints = sum(len(traj[2]) for traj in self.currentTrajectories)
        phiks = []
        for traj in self.currentTrajectories:
            # normalize contributions based on number of points in original trajectory
            phiks.append((len(traj[2]) / totalPoints) * traj[3])
        phik_list = sum(phiks)

        pdf_recon = np.zeros(self.grids.shape[0])
        for i, (phik, k_vec) in enumerate(zip(phik_list, self.ks)):
            fk_vals = np.prod(np.cos(np.pi * k_vec / self.L_list * (self.grids - self.dim_root)), axis=1)
            hk = np.sqrt(np.sum(np.square(fk_vals)) * self.dx * self.dy)
            fk_vals /= hk

            pdf_recon += phik * fk_vals

        # pdf_recon = np.maximum(pdf_recon, 0)
        # pdf_recon /= np.sum(pdf_recon) * self.dx * self.dy
        self.pdf_recon = pdf_recon

        # Trajectory optimizer setup
        dt = self.dt
        tsteps = self.tsteps
        R = np.diag([0.2, 0.2])
        Q_z = np.diag([0.5, 0.5])
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

        x0 = self.x0 - self.dim_root
        init_x_traj = bounded_spiral(x0, self.L_list, tsteps=tsteps)
        self.init_x_traj = init_x_traj
        init_u_traj = (init_x_traj[1:, :] - init_x_traj[:-1, :]) / dt

        u_traj = init_u_traj.copy()
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
        ax = self.ax
        dim_root = self.dim_root
        L_list = self.L_list
        x0 = self.x0
        pdf_vals = self.pdf_recon
        grids_x = self.grids_x
        grids_y = self.grids_y

        ax.cla()
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlim(dim_root[0], dim_root[0] + L_list[0])
        ax.set_ylim(dim_root[1], dim_root[1] + L_list[1])
        ax.minorticks_on()
        ax.tick_params(which='both', direction='in', length=6)
        ax.tick_params(which='minor', length=3, color='gray')
        ax.set_title('Map')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')

        # Plot Planning results
        if self.state == State.READY or self.state == State.MOVING:
            contour = ax.contourf(grids_x, grids_y, pdf_vals.reshape(grids_x.shape), cmap='coolwarm')

            if self.colorbar is None:
                self.colorbar = self.fig.colorbar(contour, ax=ax)
            else:
                self.colorbar.mappable.set_array(contour.collections[0].get_array())
                self.colorbar.update_normal(contour)
            # ax.plot(
            #     self.init_x_traj[:, 0] + dim_root[0],
            #     self.init_x_traj[:, 1] + dim_root[1],
            #     linestyle='-',
            #     marker='o',
            #     markersize=2,
            #     color='green',
            #     linewidth=2,
            #     alpha=1.0,
            #     label='Initial Trajectory',
            # )
            ax.plot(
                self.x_traj[:, 0] + dim_root[0],
                self.x_traj[:, 1] + dim_root[1],
                linestyle='-',
                marker='o',
                color='k',
                linewidth=2,
                alpha=1.0,
                label='Planned',
            )

        # Plot stored trajectories if not moving
        if self.state != State.MOVING:
            for traj in self.currentTrajectories:
                labels = traj[1]
                for i in range(len(traj[2]) - 1):
                    linex0, liney0 = traj[2][i]
                    linex1, liney1 = traj[2][i + 1]

                    label = labels[i]

                    # color map normalized between -1 and 1
                    color = cmap(norm(label))

                    ax.plot([linex0, linex1], [liney0, liney1], linestyle='-', linewidth=2, color=color, alpha=1.0)
            # for i, traj in enumerate(self.currentTrajectories):
            #     trajX, trajY = zip(*self.currentTrajectories[-1][2])
            #     hue = (0.3 + i * 0.1) % 1.0
            #     r, g, b = colorsys.hsv_to_rgb(hue, 0.9, 0.9)
            #     ax.plot(trajX, trajY, linestyle='-', linewidth=2, color=(r, g, b), alpha=1.0, label=traj[0])

        # If there is any recording going on, show it
        if len(self.recordBuffer) > 0:
            for i in range(len(self.recordBuffer) - 1):
                linex0, liney0 = self.recordBuffer[i]
                linex1, liney1 = self.recordBuffer[i + 1]

                label = self.recordLabels[i]

                # color map normalized between -1 and 1
                color = cmap(norm(label))

                ax.plot([linex0, linex1], [liney0, liney1], linestyle='-', linewidth=2, color=color, alpha=1.0)

        # Plot robot
        ax.plot(x0[0], x0[1], linestyle='', marker='o', markersize=10, color='C0', alpha=1.0, label='Robot')
        ax.legend(loc=1)

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
        if self.isRecording:
            if len(self.recordBuffer) < 1 or np.any(
                np.abs(self.recordBuffer[-1] - [self.position.x, self.position.y]) > 0.001
            ):
                self.recordBuffer.append(np.array([self.position.x, self.position.y]))

                # for the labels, only do dynamic labeling if the robot is providing force
                if self.state == State.MOVING:
                    if msg.actual_wrench.force.z < -5.0:
                        self.recordLabels.append(1)
                    else:
                        self.recordLabels.append(-1)
                else:
                    self.recordLabels.append(1.0)

    def timer_callback(self):
        """
        Periodic timer callback that manages the planner's state machine.

        Handles planning, visualization, and goal publishing.
        """
        # Note, no check for if robot is connected. Always verify position of real robot.
        self.draw()
        # idle is resting state, indicates no path is loaded
        if self.state == State.PLANNING:
            if len(self.currentTrajectories) > 0:
                self.plan()
                self.state = State.READY

        # if we are moving, start sending waypoints to the controller
        if self.state == State.MOVING:
            if math.dist(self.x0, self.goal) < 0.04:
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
            if self.goalIndex > len(self.x_traj) - 2:
                self.state = State.IDLE
                req = SetBool.Request()
                req.data = False
                self.future = self.ergodic_goal_toggle.call_async(req)
                rclpy.spin_until_future_complete(self, self.future)
                self.sending_goal = False
                self.goalIndex = 0

                # save trajectory that was just executed but dont write to file
                normalLabels = normalize_labels(self.recordLabels)
                coefficients = self.phiKFromTraj(self.recordBuffer - self.dim_root, normalLabels)
                self.currentTrajectories.append(('Run', normalLabels, self.recordBuffer, coefficients))
                with open('saved_data/trajectories/run.csv', 'w', newline='') as csvFile:
                    csv_writer = csv.writer(csvFile)
                    csv_writer.writerows([row.tolist() + [normalLabels[i]] for i, row in enumerate(self.recordBuffer)])
                self.recordBuffer = []
                self.recordLabels = []
                self.isRecording = False

    def user_input_loop(self):
        """Thread loop that listens for user input from the terminal."""
        while not self.state == State.SHUTDOWN:
            print('\nAvailable commands:')
            print('  shutdown - Stop app        | clear   - Clear trajectories')
            print('  moving   - Start movement  | record  - Record a trajectory')
            print('  load     - Load trajectory | plan    - Run ergodic planner')
            user_input = input('Enter state: ').strip().lower()

            # kill the app
            if user_input == 'shutdown':
                self.state = State.SHUTDOWN
                rclpy.shutdown()

            # clear trajectories in memory
            if user_input == 'clear':
                self.currentTrajectories = []

            # begin sending waypoints to follow a planned trajectory
            if user_input == 'moving':
                self.state = State.MOVING
                self.isRecording = True

            # record a new trajectory from the arm and write to a file
            if user_input == 'record':
                self.isRecording = True
                label = input(
                    'Recording. Type "name" and either "good" or "bad" to complete the capture and provide a label: '
                )

                label = label.split()

                # If this is a negative recording, label -1s instead
                if label[1] == 'bad':
                    self.recordLabels = [-1.0 * lab for lab in self.recordLabels]

                coefficients = self.phiKFromTraj(self.recordBuffer - self.dim_root, self.recordLabels)
                self.currentTrajectories.append((label[0], self.recordLabels, self.recordBuffer, coefficients))

                outputFile = 'saved_data/trajectories/' + label[0] + '-' + label[1] + '.csv'
                with open(outputFile, 'w', newline='') as csvFile:
                    csv_writer = csv.writer(csvFile)
                    csv_writer.writerows(
                        [row.tolist() + [self.recordLabels[i]] for i, row in enumerate(self.recordBuffer)]
                    )
                self.recordBuffer = []
                self.recordLabels = []
                self.isRecording = False

            # load a trajectory from a file and save the trajectory
            if user_input == 'load':
                fileName = input('Provide the name of the csv file to be loaded: ')
                points = []
                markers = []
                with open('saved_data/trajectories/' + fileName, 'r', newline='') as csvfile:
                    csv_reader = csv.reader(csvfile)
                    for row in csv_reader:
                        *point, marker = row
                        points.append(np.array(point, dtype=np.float64))
                        markers.append(float(marker))
                namePortion = fileName.split('.')
                label = namePortion[0].split('-')
                coefficients = self.phiKFromTraj(points - self.dim_root, markers)
                self.currentTrajectories.append((label[0], markers, points, coefficients))
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
