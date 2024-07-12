import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import Joy

class JoyTeleop(Node):

    def __init__(self):
        super().__init__('joy_teleop')
        self.action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.joy_subscriber = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.goal_accepted = False
        self.result_code = FollowJointTrajectory.Result.SUCCESSFUL

    def send_goal(self, position):
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server not available after waiting")
            return

        goal_msg = FollowJointTrajectory.Goal()
        point = JointTrajectoryPoint()
        point.time_from_start.sec = 1  # Reach the target position after 1 second
        point.positions = [position]
        goal_msg.trajectory.joint_names = ['slider_to_cart']
        goal_msg.trajectory.points = [point]

        self.get_logger().info(f'Sending goal: {position}')
        self.action_client.wait_for_server()
        self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            self.goal_accepted = False
        else:
            self.get_logger().info("Goal accepted")
            self.goal_accepted = True
            goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.result_code = result.error_code
        if self.result_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("SUCCEEDED result code")
        elif self.result_code == FollowJointTrajectory.Result.ABORTED:
            self.get_logger().info("Goal was aborted")
        elif self.result_code == FollowJointTrajectory.Result.CANCELED:
            self.get_logger().info("Goal was canceled")
        else:
            self.get_logger().info("Unknown result code")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_positions = feedback.desired.positions
        self.get_logger().info(f'Feedback received. Current Positions: {current_positions}')

    def joy_callback(self, msg):
        position = msg.axes[1]  # Assuming the joystick's vertical axis controls the joint position
        self.send_goal(position)

def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
