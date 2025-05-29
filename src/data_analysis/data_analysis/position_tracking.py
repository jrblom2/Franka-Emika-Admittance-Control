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
        self.xPosition_d = []
        self.yPosition_d = []
        self.zPosition_d = []
        self.time = []

        plt.ion()
        self.fig, self.axes = plt.subplots(2, 3, figsize=(10, 8))
        self.lines = []

        # First subplot
        ax = self.axes[0, 0]
        (lineX1,) = ax.plot([], [], 'r-')  # First line (red)
        (lineX2,) = ax.plot([], [], color='pink')  # Second line (pink)
        ax.set_title("X Position")
        ax.set_xlabel("Time")
        ax.set_ylabel("Position")
        self.lines.extend([lineX1, lineX2])

        # Second subplot
        ax = self.axes[0, 1]
        (lineY1,) = ax.plot([], [], 'g-')  # First line (green)
        (lineY2,) = ax.plot([], [], color='lime')  # Second line (lime)
        ax.set_title("Y Position")
        ax.set_xlabel("Time")
        ax.set_ylabel("Position")
        self.lines.extend([lineY1, lineY2])

        # Third subplot
        ax = self.axes[0, 2]
        (lineZ1,) = ax.plot([], [], 'b-')  # First line (blue)
        (lineZ2,) = ax.plot([], [], color='lightblue')  # Second line (light blue)
        ax.set_title("Z Position")
        ax.set_xlabel("Time")
        ax.set_ylabel("Position")
        self.lines.extend([lineZ1, lineZ2])

        self.counter = 0

    def listener_callback(self, msg):
        self.xPosition.append(msg.position.position.x)
        self.yPosition.append(msg.position.position.y)
        self.zPosition.append(msg.position.position.z)
        self.xPosition_d.append(msg.position_d.position.x)
        self.yPosition_d.append(msg.position_d.position.y)
        self.zPosition_d.append(msg.position_d.position.z)
        self.time.append(self.counter / 20)
        self.counter += 1

    def timer_callback(self):
        self.lines[0].set_data(self.time, self.xPosition)
        self.lines[1].set_data(self.time, self.xPosition_d)
        self.axes[0, 0].relim()
        self.axes[0, 0].autoscale_view()

        self.lines[2].set_data(self.time, self.yPosition)
        self.lines[3].set_data(self.time, self.yPosition_d)
        self.axes[0, 1].relim()
        self.axes[0, 1].autoscale_view()

        self.lines[4].set_data(self.time, self.zPosition)
        self.lines[5].set_data(self.time, self.zPosition_d)
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
