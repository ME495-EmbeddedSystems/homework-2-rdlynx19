import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist, Vector3, PoseStamped
import rclpy.time

from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from sensor_msgs.msg import JointState
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry


def turtle_twist(linear_velocity, angular_velocity):
    """Create a twist velocity suitable for a turtle

    Args:
        linear_vel (list of floats): the linear velocities
        angular_vel (list of floats): the angular velocities

    Returns:
        Twist - a 2D twist object corresponding to linear/angular velocity
    """
    return Twist(
        linear=Vector3(
            x=linear_velocity[0], y=linear_velocity[1], z=linear_velocity[2]
        ),
        angular=Vector3(
            x=angular_velocity[0], y=angular_velocity[1], z=angular_velocity[2]
        ),
    )


class Turtle_Robot(Node):
    """
    Turtle Robot Node
    """

    def __init__(self):
        super().__init__("turtle_robot")
        # static broadcaster for the world frame
        self.static_world_broadcaster = StaticTransformBroadcaster(self)

        world_odom_tf = TransformStamped()
        world_odom_tf.header.stamp = self.get_clock().now().to_msg()
        world_odom_tf.header.frame_id = "world"
        world_odom_tf.child_frame_id = "odom"

        world_odom_tf.transform.translation.x = 2.0
        self.static_world_broadcaster.sendTransform(world_odom_tf)
        self.get_logger().info("Static Transfrom: world->odom")

        # transformation from odom to base_link
        self.odom_broadcaster = TransformBroadcaster(self)
        self.turtle_tmr = self.create_timer(1 / 100, self.turtle_tmr_callback)

        # joint state publisher
        self.joint_state_publisher = self.create_publisher(
            JointState, "joint_states", 1
        )
        self.joint_state_tmr = self.create_timer(1 / 100, self.joint_state_tmr_callback)

        # turtle pose subscriber
        self.turtlesim_pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.turtlesim_pose_callback, 1
        )

        # odom message publisher
        self.odom_publisher = self.create_publisher(Odometry, "odom", 1)

        # cmd_vel publisher
        self.current_velocity = Twist()  # creating a twist class variable
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 1)
        self.cmd_vel_tmr = self.create_timer(1 / 100, self.cmd_vel_tmr_callback)

        # goal pose subscriber
        self.goal_pose = PoseStamped()
        self.goal_pose_subscriber = self.create_subscription(
            PoseStamped, "goal_pose", self.goal_pose_callback, 1
        )

    def turtle_tmr_callback(self):
        odom_base_tf = TransformStamped()
        odom_base_tf.header.stamp = self.get_clock().now().to_msg()
        odom_base_tf.header.frame_id = "odom"
        odom_base_tf.child_frame_id = "base_link"
        odom_base_tf.transform.translation.z = 0.5        

        self.odom_broadcaster.sendTransform(odom_base_tf)

    def joint_state_tmr_callback(self):
        robot_joint_states = JointState()
        robot_joint_states.header.stamp = self.get_clock().now().to_msg()
        robot_joint_states.header.frame_id = "odom"
        robot_joint_states.name = ["base_platform", "base_stem", "stem_wheel"]
        robot_joint_states.position = [-1.57, 0, 1.57]

        self.joint_state_publisher.publish(robot_joint_states)

    def cmd_vel_tmr_callback(self):
        # publish cmd_vel messages here
        # use goal pose to calculate apt cmd_vel messages
        pass

    def turtlesim_pose_callback(self, turtlesim_pose_msg):
        # compute the twist velocity to be published on odom topic

        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.twist.twist = self.current_velocity
        odom_msg.pose.pose.position.x = turtlesim_pose_msg.x
        odom_msg.pose.pose.position.y = turtlesim_pose_msg.y
        odom_msg.pose.pose.position.z = 0.0
        
        # current_turtlesim_point = Point(pose_msg.x, pose_msg.y, 0)
        # odom_msg.pose.pose.point = current_turtlesim_point
        ###### complete this

    def goal_pose_callback(self, goal_pose_msg):
        # command velocity to send robot to goal pose
        self.goal_pose = goal_pose_msg


def turtle_robot_start(args=None):
    rclpy.init(args=args)
    node = Turtle_Robot()
    rclpy.spin(node)
    rclpy.shutdown()
