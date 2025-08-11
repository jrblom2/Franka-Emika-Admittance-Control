import rclpy
from geometry_msgs.msg._point import Point
from rclpy.node import Node

from data_interfaces.msg import Robot
from ergodic.ergodic_wrapper import ErgodicWrapper
import numpy as np


class ErgodicPlanner(Node):

    def __init__(self):
        super().__init__('ergodic_planner')
        self.robot_state_subscription = self.create_subscription(Robot, 'robot_data', self.listener_callback, 10)
        self.ergodic_goal_publisher = self.create_publisher(Point, 'ergodic_goal', 10)
        self.robot_state_subscription  # prevent unused variable warning
        self.timer = self.create_timer(3.0, self.timer_callback)
        self.position = None
        self.planner = ErgodicWrapper()
        self.hasPlan = False
        self.currentGoalIndex = 0

    def listener_callback(self, msg):
        self.position = msg.position.position
        self.planner.x0 = np.array([self.position.x, self.position.y])

    def timer_callback(self):
        if self.position is not None:
            # only continue if we have received a position from the robot
            if not self.hasPlan:
                self.planner.plan()
                self.hasPlan = True
            # once we have a plan loaded, always draw
            else:
                self.planner.draw()

            msg = Point()
            msg = self.position
            self.ergodic_goal_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    ergodic_planner = ErgodicPlanner()
    rclpy.spin(ergodic_planner)


if __name__ == '__main__':
    main()
