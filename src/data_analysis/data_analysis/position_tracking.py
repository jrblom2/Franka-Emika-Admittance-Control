import rclpy
from rclpy.node import Node

from data_interfaces.msg import Robot

import matplotlib.pyplot as plt


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Robot, 'robot_data', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(1, self.timer_callback)
        self.xForce = []
        self.axis = []

        plt.ion()
        self.fig, self.ax = plt.subplots()
        (self.line,) = self.ax.plot([], [], 'r-')
        self.ax.set_title("Force in X direction")
        self.ax.set_xlabel("Time")
        self.ax.set_ylabel("Force")
        self.counter = 0

    def listener_callback(self, msg):
        self.xForce.append(msg.wrench.force.x)
        self.axis.append(self.counter / 20)
        self.counter += 1

    def timer_callback(self):
        self.line.set_data(self.axis, self.xForce)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.1)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
