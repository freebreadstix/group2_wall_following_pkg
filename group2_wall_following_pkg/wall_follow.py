import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import sys


class WallFollow(Node):

    def __init__(self):
        # Here we have the class constructor
        # call the class constructor
        super().__init__('wall_follow')
        cb_group = ReentrantCallbackGroup()
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create the subscriber object
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.read_laser, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=cb_group)
        # prevent unused variable warning
        self.subscriber

        self.timer_period = 0.1
        # define the variable to save the received info
        self.laser_right = 0
        self.laser_front = 0
        self.min_laser = (0, 0)
        self.front_wall = False
        self.finish = False
        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def read_laser(self, msg):
        ml = 0
        for i, val in enumerate(msg.ranges):
            if msg.ranges[ml] < 5:
                ml = i
            elif val < msg.ranges[ml] and val > 5:
                ml = i
        self.min_laser = (ml, msg.ranges[ml])

        self.get_logger().info('Length: "%s", Furthest: "%s"' %
            (str(len(msg.ranges)), str(self.min_laser)))

    def motion(self):
        # print the data
        #self.get_logger().info('Right: "%s", Front: "%s", Nearest: "%s"' %
            #(str(self.laser_right), str(self.laser_front), str(self.min_laser)))
        # Logic of move

            # if self.laser_right > 0.27:
            #    self.cmd.linear.x = 0.05
            #    self.cmd.angular.z = -0.15
            # elif self.laser_right > 0.26:
            #    self.cmd.linear.x = 0.05
            #    self.cmd.angular.z = -0.1
            # elif self.laser_right < 0.24:
            #    self.cmd.linear.x = 0.05
            #    self.cmd.angular.z = 0.1
            # elif self.laser_right < 0.23:
            #    self.cmd.linear.x = 0.05
            #    self.cmd.angular.z = 0.15
            # else:
            #    self.cmd.linear.x = 0.05
            #    self.cmd.angular.z = 0.0
            # Publishing the cmd_vel values to topipc
        self.publisher_.publish(self.cmd)
        if self.finish:
            self.cmd.angular.z = 0.0
            self.cmd.linear.x = 0.0
            self.publisher_.publish(self.cmd)
            self.destroy_timer(self.timer)


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    wall_follow = WallFollow()
    executor = MultiThreadedExecutor()
    executor.add_node(wall_follow)
    
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    executor.spin()
    # Explicity destroy the node
    wall_follow.destroy_node()
    # shutdown the ROS communication
    executor.shutdown()


if __name__ == '__main__':
    main()
