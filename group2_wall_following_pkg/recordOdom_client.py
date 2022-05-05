import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from custom_interfaces.action import OdomRecord


class MyActionClient(Node):

    def __init__(self):
        super().__init__('record_odom_client')
        self._action_client = ActionClient(self, OdomRecord, 'record_odom')

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
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        cur_total = feedback_msg.feedback.current_total
        self.get_logger().info(
            'Received feedback: {0}'.format(cur_total))


def main(args=None):
    rclpy.init(args=args)

    action_client = MyActionClient()

    future = action_client.send_goal()
    executor = MultiThreadedExecutor()

    rclpy.spin(action_client, executor=executor)


if __name__ == '__main__':
    main()
