import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from custom_interfaces.action import OdomRecord
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import math
import numpy as np


class MyActionServer(Node):

    def __init__(self):
        super().__init__('record_odom')
        cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self, OdomRecord, 'record_odom', self.execute_callback, callback_group=cb_group)
        self.subscriber = self.create_subscription(
            Odometry, '/odom', self.read_odom, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=cb_group)
        # prevent unused variable warning
        self.feedback_msg = OdomRecord.Feedback()

        self.subscriber
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.total_distance = 0.0
        self.odom_list = []
        self.record = False

    def read_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        theta = 2*math.atan2(z, w)
        self.theta = np.degrees(theta)

        if self.record:
            self.odom_list += [(self.x, self.y, self.theta)]
            # self.get_logger().info(str(self.odom_list[-1]))
            if len(self.odom_list) > 1:
                d_increment = math.sqrt((self.odom_list[-1][0] - self.odom_list[-2][0])**2 +
                                        (self.odom_list[-1][1] - self.odom_list[-2][1])**2)
            else:
                d_increment = 0
            self.feedback_msg.current_total += d_increment

            # self.get_logger().info('Total distance travelled: ' + str(
            # self.feedback_msg.current_total))

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.record = True

        while self.record:
            # self.get_logger().info(str(self.feedback_msg.current_total))
            #last_pos = self.odom_list[-1]
            # self.get_logger().info(str(last_pos))
            #pos = Point32()
            #pos.x = last_pos[0]
            #pos.y = last_pos[1]
            #pos.z = last_pos[2]
            #list_of_odoms += [pos]
            goal_handle.publish_feedback(self.feedback_msg)
            # time.sleep(1)
            # 5 is the dist
            if (len(self.odom_list) > 1 and self.feedback_msg.current_total > 4 and math.sqrt((self.odom_list[-1][0] - self.odom_list[0][0])**2 +
                                                                                              (self.odom_list[-1][1] - self.odom_list[0][1])**2) < 0.2):  # and
                # self.odom_list[-1][0] == self.odom_list[0][0] and self.odom_list[-1][1] >= self.odom_list[0][1]):
                self.get_logger().info('test')
                self.record = False

        self.get_logger().info(str(len(self.odom_list)))
        result = OdomRecord.Result()
        result.list_of_odoms = [Point32(x=i[0], y=i[1], z=i[2])
                                for i in self.odom_list]
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)

    my_action_server = MyActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(my_action_server)
    executor.spin()
    # rclpy.spin(my_action_server)


if __name__ == '__main__':
    main()
