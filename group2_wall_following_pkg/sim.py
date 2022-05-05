import rclpy
from custom_interfaces.srv import FindWall
from custom_interfaces.action import OdomRecord
from rclpy.action import ActionClient
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


class Sim(Node):

    def __init__(self):
        # Here we have the class constructor
        # call the class constructor
        super().__init__('sim')
        cb_group = ReentrantCallbackGroup()
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create the subscriber object
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.read_laser, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=cb_group)
        # prevent unused variable warning
        self.subscriber
        # create the service client object
        # defines the name and type of the service server we will work with.
        self._action_client = ActionClient(self, OdomRecord, 'record_odom')
        self.client = self.create_client(FindWall, 'find_wall')
        # checks once per second if a service matching the type and name of the client is available.
        while not self.client.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            self.get_logger().info('service not available, waiting again...')

        # create a Empty request
        self.req = FindWall.Request()
        # define the timer period for 0.5 seconds
        self.timer_period = 0.1
        # define the variable to save the received info
        self.laser_right = 0
        self.laser_front = 0
        self.min_laser = (0, 0)
        self.front_wall = False
        self.finish = False
        # create a Twist message
        self.cmd = Twist()

    def send_goal(self):
        goal_msg = OdomRecord.Goal()
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.list_of_odoms))
        self.finish = True

    def feedback_callback(self, feedback_msg):
        cur_total = feedback_msg.feedback.current_total
        self.get_logger().info(
            'Received feedback: {0}'.format(cur_total))

    def send_request(self):

        self.future = self.client.call_async(self.req)
        # to print in the console

    def read_laser(self, msg):
        ml = 0
        for i, val in enumerate(msg.ranges):
            if val < msg.ranges[ml]:
                ml = i
        self.min_laser = (ml, msg.ranges[ml])

        self.laser_right = msg.ranges[90]
        self.laser_front = msg.ranges[180]

    def motion(self):
        # print the data
        # self.get_logger().info('Right: "%s", Front: "%s", Nearest: "%s"' %
        # (str(self.laser_right), str(self.laser_front), str(self.min_laser)))
        # Logic of move

        if self.front_wall and self.laser_front < 0.8:
            self.cmd.linear.x = 0.05
            self.cmd.angular.z = 0.4
        elif self.laser_front < 0.4:
            self.front_wall = True
            self.cmd.linear.x = 0.05
            self.cmd.angular.z = 0.2
        elif self.front_wall and self.laser_front >= 0.8:
            self.front_wall = False
            self.cmd.linear.x = 0.08
            self.cmd.angular.z = 0.0
        elif self.laser_front > 10:
            self.cmd.linear.x = -1.0
            self.cmd.angular.z = 0
        elif not self.front_wall:
            if self.min_laser[0] < 190 and self.min_laser[1] > .26:
                if self.min_laser[0] < 100:
                    self.cmd.angular.z = -0.2
                    self.cmd.linear.x = 0.08
                else:
                    self.cmd.angular.z = 0.0
                    self.cmd.linear.x = 0.08
            elif self.min_laser[0] < 190 and self.min_laser[1] < .2:
                if self.min_laser[0] > 80:
                    self.cmd.angular.z = 0.2
                    self.cmd.linear.x = 0.08
                else:
                    self.cmd.angular.z = 0.0
                    self.cmd.linear.x = 0.08
            elif self.min_laser[0] >= 190:
                if self.min_laser[0] < 270:
                    self.cmd.angular.z = -0.32
                    self.cmd.linear.x = 0.08
                else:
                    self.cmd.angular.z = 0.0
                    self.cmd.linear.x = 0.08
            else:
                if self.min_laser[0] > 100:
                    self.cmd.angular.z = 0.1
                    self.cmd.linear.x = 0.08
                elif self.min_laser[0] < 80:
                    self.cmd.angular.z = -0.1
                    self.cmd.linear.x = 0.08

                else:
                    self.cmd.angular.z = 0.0
                    self.cmd.linear.x = 0.08
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
    sim = Sim()
    fut = sim.send_goal()
    executor = MultiThreadedExecutor()
    executor.add_node(sim)
    # spin_thread = Thread(target=rclpy.spin, args=(sim,))
    # spin_thread.start()
    # run the send_request() method
    sim.send_request()

    while rclpy.ok():
        # pause the program execution, waits for a request to kill the node (ctrl+c)
        executor.spin_once()
        if sim.future.done():
            try:
                # checks the future for a response from the service
                # while the system is running.
                # If the service has sent a response, the result will be written
                # to a log message.
                response = sim.future.result()
            except Exception as e:
                # Display the message on the console
                sim.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                # Display the message on the console
                sim.get_logger().info(
                    'Response state %r' % (response.wallfound,))
            break

    # if response.wallfound:
    sim.get_logger().info('Wall found, beginning motion')
    sim.timer = sim.create_timer(sim.timer_period, sim.motion)
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    executor.spin()
    # Explicity destroy the node
    sim.destroy_node()
    # shutdown the ROS communication
    executor.shutdown()


if __name__ == '__main__':
    main()
