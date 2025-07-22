import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class GestureToCmdVel(Node):
    def __init__(self):
        super().__init__('gesture_to_cmd_vel')

        # Declare and get robot name parameter
        self.declare_parameter('robot_name', 'robot1')
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        # Topic subscriptions/publishing
        self.subscription = self.create_subscription(
            String,
            f'/{robot_name}_hand_action',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(
            Twist,
            f'/{robot_name}/cmd_vel',
            10)

        self.get_logger().info(f'[{robot_name}] GestureToCmdVel node started.')

    def listener_callback(self, msg):
        action = msg.data.lower()
        cmd = Twist()

        if action == "move_forward":
            cmd.linear.x = 0.3
        elif action == "move_backward":
            cmd.linear.x = -0.3
        elif action == "turn_left":
            cmd.angular.z = 1.0
        elif action == "turn_right":
            cmd.angular.z = -1.0
        elif action == "stop":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            self.get_logger().warn(f"Unknown action: {action}")
            return

        self.publisher.publish(cmd)
        self.get_logger().info(f'Published velocity command for action: {action}')

def main(args=None):
    rclpy.init(args=args)
    node = GestureToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()