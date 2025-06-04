import rclpy
from rclpy.node import Node
import numpy as np

from data_interfaces.msg import Robot
import matplotlib.pyplot as plt


class MinimalSubscriberTorque(Node):

    def __init__(self):
        super().__init__('minimal_subscriber_torque')
        self.subscription = self.create_subscription(Robot, 'robot_data', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(1, self.timer_callback)
        self.joint1d = []
        self.joint1r = []
        self.joint1o = []

        self.time = []

        plt.ion()
        self.fig, self.axes = plt.subplots(3, 3, figsize=(10, 8))
        self.fig.suptitle('reported = libFranka desired | observed = reported t - gravity t', fontsize=16)
        self.lines = []

        # Joint 1
        ax = self.axes[0, 0]
        (line1d,) = ax.plot([], [], 'r-', label='1 commanded')
        (line1r,) = ax.plot([], [], 'g-', label='1 reported')
        (line1o,) = ax.plot([], [], 'b-', label='1 observed')
        ax.set_title("Joint 1")
        ax.set_xlabel("Time (seconds)")
        ax.set_ylabel("Torque (N/meters)")
        ax.legend()
        self.lines.extend([line1d, line1r, line1o])

        self.counter = 0

    def listener_callback(self, msg):
        self.joint1d.append(msg.torques_desired[0])
        self.joint1r.append(msg.torques_observed[0])
        self.joint1o.append(msg.torques_gravity[0])

        self.time.append(self.counter / 100)
        self.counter += 1

    def timer_callback(self):
        self.lines[0].set_data(self.time, self.joint1d)
        self.lines[1].set_data(self.time, self.joint1r)
        self.lines[2].set_data(self.time, self.joint1o)
        self.axes[0, 0].relim()
        self.axes[0, 0].autoscale_view()

        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.1)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriberTorque()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
