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
            if msg.ranges[ml] < .10:
                ml = i
            elif val < msg.ranges[ml] and val > 0.1:
                ml = i
        self.min_laser = (ml, msg.ranges[ml])
        self.laser_right = msg.ranges[135]

        # self.get_logger().info('Nearest: "%s"' %
        #     (str(self.min_laser)))

    def motion(self):
        # print the data
        #self.get_logger().info('Right: "%s", Front: "%s", Nearest: "%s"' %
            #(str(self.laser_right), str(self.laser_front), str(self.min_laser)))
        # Logic of move
        fwd_spd = 0.2
        ang_spd = 0.8
        
        if self.min_laser[0] < 405:
            self.get_logger().info(str(self.min_laser))


        # if self.laser_right > 1:
        #     self.cmd.linear.x = 0.0
        #     self.cmd.angular.z = 0.0
        #     self.destroy_timer(self.timer)
        if self.min_laser[0] < 135:
            if self.min_laser[1] < .3:
                self.cmd.angular.z = 0.0
                self.cmd.linear.x = fwd_spd
            elif self.min_laser[1] > .3:
                self.cmd.angular.z = ang_spd
                self.cmd.linear.x = fwd_spd
        elif self.min_laser[0] > 135 and self.min_laser[0] < 405:
            if self.min_laser[1] < .3:
                self.cmd.angular.z = -ang_spd
                self.cmd.linear.x = fwd_spd
            elif self.min_laser[1] > .3:
                self.cmd.angular.z = 0.0
                self.cmd.linear.x = fwd_spd

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
