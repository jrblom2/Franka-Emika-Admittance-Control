import rclpy
from rclpy.node import Node
import time

from data_interfaces.msg import Robot
import matplotlib.pyplot as plt


class MinimalSubscriberTorque(Node):

    def __init__(self):
        super().__init__('minimal_subscriber_torque')
        self.subscription = self.create_subscription(Robot, 'robot_data', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(1, self.timer_callback)

        self.num_joints = 7
        self.jointd = [[] for _ in range(self.num_joints)]
        self.jointr = [[] for _ in range(self.num_joints)]
        self.jointo = [[] for _ in range(self.num_joints)]
        self.time = []
        self.start = None

        plt.ion()
        self.fig, self.axes = plt.subplots(3, 3, figsize=(12, 9))
        self.fig.suptitle('reported = libFranka desired | observed = reported t - gravity t', fontsize=14)

        self.lines = []

        for i in range(self.num_joints):
            row, col = divmod(i, 3)
            ax = self.axes[row, col]
            (lined,) = ax.plot([], [], 'r-', label=f'{i+1} commanded')
            (liner,) = ax.plot([], [], 'g-', label=f'{i+1} reported')
            (lineo,) = ax.plot([], [], 'b-', label=f'{i+1} observed')
            ax.set_title(f"Joint {i+1}")
            ax.set_xlabel("Time (seconds)")
            ax.set_ylabel("Torque (N·m)")
            ax.legend()
            self.lines.extend([lined, liner, lineo])

        self.counter = 0

    def listener_callback(self, msg):
        if self.start is None:
            self.start = time.time()

        for i in range(self.num_joints):
            self.jointd[i].append(msg.torques_desired[i])
            self.jointr[i].append(msg.torques_observed[i])
            self.jointo[i].append(msg.torques_gravity[i])
        self.time.append(time.time() - self.start)
        self.counter += 1

    def timer_callback(self):
        for i in range(self.num_joints):
            t = self.time
            self.lines[i * 3].set_data(t, self.jointd[i])
            self.lines[i * 3 + 1].set_data(t, self.jointr[i])
            self.lines[i * 3 + 2].set_data(t, self.jointo[i])

            row, col = divmod(i, 3)
            ax = self.axes[row, col]
            ax.relim()
            ax.autoscale_view()

        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.1)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriberTorque()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
