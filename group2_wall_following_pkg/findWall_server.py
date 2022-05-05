# import the Twist module from geometry_msgs messages interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
# import the MyCustomServiceMessage module from custom_interfaces_service interface
from custom_interfaces.srv import FindWall
# import the ROS2 python client libraries
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import rclpy
from rclpy.node import Node
from time import sleep


class Service(Node):

    def __init__(self):
        # Here we have the class constructor

        # call the class constructor to initialize the node as service_stop
        super().__init__('find_wall')
        cb_group = ReentrantCallbackGroup()
        # create the service server object
        # defines the type, name and callback function
        self.srv = self.create_service(
            FindWall, 'find_wall', self.FindWall_callback, callback_group=cb_group)
        # create the publisher object
        # in this case the publisher will publish on /cmd_vel topic with a queue size of 10 messages.
        # use the Twist module
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.read_laser, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=cb_group)

        self.subscriber

        self.cmd = Twist()
        self.wall = 0
        self.min_laser = (0, 0)
        self.laser_front = 0

    def read_laser(self, msg):
        ml = 0
        for i, val in enumerate(msg.ranges):
            if val < msg.ranges[ml]:
                ml = i
        self.min_laser = (ml, msg.ranges[ml])
        self.laser_front = msg.ranges[180]

    def FindWall_callback(self, request, response):
        # The callback function recives the self class parameter,
        # received along with two parameters called request and response
        # - receive the data by request
        # - return a result as response

        self.get_logger().info('Min: "%s", Front: "%s"' %
                               (str(self.min_laser), str(self.laser_front)))
        if self.min_laser[0] < 180:
            while self.min_laser[0] < 180:
                self.cmd.angular.z = -0.3
                self.cmd.linear.x = 0.0
                self.publisher_.publish(self.cmd)
            self.cmd.angular.z = 0.0
            self.cmd.linear.x = 0.0
            self.publisher_.publish(self.cmd)
        elif self.min_laser[0] > 180:
            while self.min_laser[0] > 180:
                self.cmd.angular.z = 0.3
                self.cmd.linear.x = 0.0
                self.publisher_.publish(self.cmd)
            self.cmd.angular.z = 0.0
            self.cmd.linear.x = 0.0
            self.publisher_.publish(self.cmd)
        self.get_logger().info('Min: "%s", Front: "%s"' %
                               (str(self.min_laser), str(self.laser_front)))

        if self.laser_front < 0.25:
            while self.laser_front < 0.25:
                self.cmd.angular.z = 0.0
                self.cmd.linear.x = -0.05
                self.publisher_.publish(self.cmd)
            self.cmd.angular.z = 0.0
            self.cmd.linear.x = 0.0
            self.publisher_.publish(self.cmd)
        elif self.laser_front > 0.25:
            while self.laser_front > 0.25:
                self.cmd.angular.z = 0.0
                self.cmd.linear.x = 0.05
                self.publisher_.publish(self.cmd)
            self.cmd.angular.z = 0.0
            self.cmd.linear.x = 0.0
            self.publisher_.publish(self.cmd)
        self.get_logger().info('Min: "%s", Front: "%s"' %
                               (str(self.min_laser), str(self.laser_front)))
        if self.min_laser[0] > 90:
            while self.min_laser[0] > 90:
                self.cmd.angular.z = 0.3
                self.cmd.linear.x = 0.0
                self.publisher_.publish(self.cmd)
            self.cmd.angular.z = 0.0
            self.cmd.linear.x = 0.0
            self.publisher_.publish(self.cmd)

        #self.cmd.angular.z = 0.4
        # self.publisher_.publish(self.cmd)
        #runtime = self.min_laser[0] - 180.0
        # if 180.0 > self.min_laser[0]:
        #    runtime += 360.0
        # sleep(runtime/8.4)
        #self.cmd.angular.z = 0.0
        # self.publisher_.publish(self.cmd)

        # if self.min_laser[1] < 0.2:
        #    self.cmd.linear.x = -0.2
        #    self.publisher_.publish(self.cmd)
        #    sleep((0.23 - self.min_laser[1]) / 0.064)
        #    self.cmd.linear.x = 0.0
        #    self.publisher_.publish(self.cmd)
        # elif self.min_laser[1] > 0.3:
        #    self.cmd.linear.x = 0.2
        #    self.publisher_.publish(self.cmd)
        #    sleep((self.min_laser[1] - 0.23) / 0.084)
        #    self.cmd.linear.x = 0.0
        #    self.publisher_.publish(self.cmd)

        #self.cmd.angular.z = 0.4
        # self.publisher_.publish(self.cmd)
        # sleep(10.6)
        #self.cmd.angular.z = 0.0
        # self.publisher_.publish(self.cmd)

        self.get_logger().info('Min: "%s", Front: "%s"' %
                               (str(self.min_laser), str(self.laser_front)))

        response.wallfound = True

        # return the response parameter
        return response

    # def rotate(self):
    #    self.get_logger().info('Min: "%s", Front: "%s"' %
    #                           (str(self.min_laser), str(self.laser_front)))
    #    if not ((self.min_laser[1] - 0.005 <= self.laser_front) and (self.laser_front <= self.min_laser[1] + 0.005)):
    #        self.cmd.angular.z = 0.7
    #        self.publisher_.publish(self.cmd)
    #    else:
    #        self.cmd.angular.z = 0.0
    #        self.publisher_.publish(self.cmd)
    #        self.destroy_timer(self.timer)


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    service = Service()
    executor = MultiThreadedExecutor()
    executor.add_node(service)
    executor.spin()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    # rclpy.spin(service)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
