import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand
from sensor_msgs.msg import JointState


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # Declare a configurable parameter (defaults to 'px100')
        self.declare_parameter('robot_name', 'px100')
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        # Build topic names dynamically from robot_name
        self.arm_pub = self.create_publisher(
            JointGroupCommand, f'/{robot_name}/commands/joint_group', 10
        )
        self.gripper_pub = self.create_publisher(
            JointSingleCommand, f'/{robot_name}/commands/joint_single', 10
        )
        self.joint_sub = self.create_subscription(
            JointState, f'/{robot_name}/joint_states', self.joint_state_callback, 10
        )

        self.current_joints = {}

        # Waypoints: [waist, shoulder, elbow, wrist_angle] in radians
        self.poses = [
            [0.0, 0.0, 0.0, 0.0],       # Vertical upright
            [0.5, -0.5, 0.5, 0.0],     # Lift & turn left
            [-0.5, -0.5, 0.5, 0.0],    # Lift & turn right
            [0.0, -1.88, 1.5, 0.8],    # Sleep pose
        ]
        self.pose_index = 0

        self.timer = self.create_timer(2.0, self.send_next_pose)
        self.get_logger().info(f'Arm Controller started for robot: "{robot_name}"')

    def joint_state_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.current_joints[name] = round(pos, 3)

    def send_next_pose(self):
        target = self.poses[self.pose_index]

        msg = JointGroupCommand()
        msg.name = 'arm'
        msg.cmd = target
        self.arm_pub.publish(msg)

        self.get_logger().info(f'Sent pose {self.pose_index}: {target}')
        self.pose_index = (self.pose_index + 1) % len(self.poses)


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
