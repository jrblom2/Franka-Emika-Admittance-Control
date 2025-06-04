import rclpy
from rclpy.node import Node
import numpy as np

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
        self.positionErrorMag = []
        self.roll = []
        self.pitch = []
        self.yaw = []
        self.xForce = []
        self.yForce = []
        self.zForce = []

        self.time = []

        plt.ion()
        self.fig, self.axes = plt.subplots(2, 3, figsize=(10, 8))
        self.lines = []

        # X subplot
        ax = self.axes[0, 0]
        (lineX1,) = ax.plot([], [], 'r-', label='X Actual')
        (lineX2,) = ax.plot([], [], color='pink', label='X Target')
        ax.set_title("X Position")
        ax.set_xlabel("Time (seconds)")
        ax.set_ylabel("Position (meters)")
        ax.legend()
        self.lines.extend([lineX1, lineX2])

        # Y subplot
        ax = self.axes[0, 1]
        (lineY1,) = ax.plot([], [], 'g-', label='Y Actual')
        (lineY2,) = ax.plot([], [], color='lime', label='Y Target')
        ax.set_title("Y Position")
        ax.set_xlabel("Time (seconds)")
        ax.set_ylabel("Position (meters)")
        ax.legend()
        self.lines.extend([lineY1, lineY2])

        # Z subplot
        ax = self.axes[0, 2]
        (lineZ1,) = ax.plot([], [], 'b-', label='Z Actual')
        (lineZ2,) = ax.plot([], [], color='lightblue', label='Z Target')
        ax.set_title("Z Position")
        ax.set_xlabel("Time (seconds)")
        ax.set_ylabel("Position (meters)")
        ax.legend()
        self.lines.extend([lineZ1, lineZ2])

        # Roll Pitch Yaw subplot
        ax = self.axes[1, 0]
        (lineR,) = ax.plot([], [], 'r-', label='Roll')
        (lineP,) = ax.plot([], [], 'g-', label='Pitch')
        (lineY,) = ax.plot([], [], 'b-', label='Yaw')
        ax.set_title("Roll Pitch Yaw")
        ax.set_xlabel("Time (seconds)")
        ax.set_ylabel("Position (radians)")
        ax.set_ylim(-3.14, 3.14)
        ax.legend()
        self.lines.extend([lineR, lineP, lineY])

        # Position Error magnitude
        ax = self.axes[1, 1]
        (lineErrorMag,) = ax.plot([], [], 'r-', label='Error Magnitude')
        ax.set_title("Magnitude of Position Error")
        ax.set_xlabel("Time (seconds)")
        ax.set_ylabel("Magnitude (meters)")
        ax.legend()
        self.lines.extend([lineErrorMag])

        # Commanded Wrench on EE
        ax = self.axes[1, 2]
        (lineXWrench,) = ax.plot([], [], 'r-', label='X')
        (lineYWrench,) = ax.plot([], [], 'g-', label='Y')
        (lineZWrench,) = ax.plot([], [], 'b-', label='Z')
        ax.set_title("Commanded Acceleration")
        ax.set_xlabel("Time (seconds)")
        ax.set_ylabel("Magnitude (M/S^2)")
        ax.legend()
        self.lines.extend([lineXWrench, lineYWrench, lineZWrench])

        self.counter = 0

    def listener_callback(self, msg):
        self.xPosition.append(msg.position.position.x)
        self.yPosition.append(msg.position.position.y)
        self.zPosition.append(msg.position.position.z)
        self.xPosition_d.append(msg.position_d.position.x)
        self.yPosition_d.append(msg.position_d.position.y)
        self.zPosition_d.append(msg.position_d.position.z)
        self.roll.append(msg.position.orientation.x)
        self.pitch.append(msg.position.orientation.y)
        self.yaw.append(msg.position.orientation.z)

        actual = np.array([msg.position.position.x, msg.position.position.y, msg.position.position.z])
        desired = np.array([msg.position_d.position.x, msg.position_d.position.y, msg.position_d.position.z])
        error = np.linalg.norm(actual - desired)
        self.positionErrorMag.append(error)

        self.xForce.append(msg.wrench.force.x)
        self.yForce.append(msg.wrench.force.y)
        self.zForce.append(msg.wrench.force.z)

        self.time.append(self.counter / 100)
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

        self.lines[6].set_data(self.time, self.roll)
        self.lines[7].set_data(self.time, self.pitch)
        self.lines[8].set_data(self.time, self.yaw)
        self.axes[1, 0].relim()
        self.axes[1, 0].autoscale_view(scaley=False)

        self.lines[9].set_data(self.time, self.positionErrorMag)
        self.axes[1, 1].relim()
        self.axes[1, 1].autoscale_view()

        self.lines[10].set_data(self.time, self.xForce)
        self.lines[11].set_data(self.time, self.yForce)
        self.lines[12].set_data(self.time, self.zForce)
        self.axes[1, 2].relim()
        self.axes[1, 2].autoscale_view()

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
