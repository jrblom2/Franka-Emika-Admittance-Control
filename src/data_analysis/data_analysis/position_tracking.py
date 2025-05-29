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
        self.xPosition = []
        self.yPosition = []
        self.zPosition = []
        self.time = []

        plt.ion()
        self.fig, self.axes = plt.subplots(2, 3, figsize=(10, 8))
        self.lines = []

        ax = self.axes[0, 0]
        (lineX,) = ax.plot([], [], 'r-')
        ax.set_title("X Position")
        ax.set_xlabel("Time")
        ax.set_ylabel("Position")
        self.lines.append(lineX)

        ax = self.axes[0, 1]
        (lineY,) = ax.plot([], [], 'g-')
        ax.set_title("Y Position")
        ax.set_xlabel("Time")
        ax.set_ylabel("Position")
        self.lines.append(lineY)

        ax = self.axes[0, 2]
        (lineZ,) = ax.plot([], [], 'b-')
        ax.set_title("Z Position")
        ax.set_xlabel("Time")
        ax.set_ylabel("Position")
        self.lines.append(lineZ)

        self.counter = 0

    def listener_callback(self, msg):
        self.xPosition.append(msg.position.position.x)
        self.yPosition.append(msg.position.position.y)
        self.zPosition.append(msg.position.position.z)
        self.time.append(self.counter / 20)
        self.counter += 1

    def timer_callback(self):
        self.lines[0].set_data(self.time, self.xPosition)
        self.axes[0, 0].relim()
        self.axes[0, 0].autoscale_view()

        self.lines[1].set_data(self.time, self.yPosition)
        self.axes[0, 1].relim()
        self.axes[0, 1].autoscale_view()

        self.lines[2].set_data(self.time, self.zPosition)
        self.axes[0, 2].relim()
        self.axes[0, 2].autoscale_view()

        self.fig.tight_layout()
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
