"""The Turtle Robot Node."""
import math

from geometry_msgs.msg import PoseStamped, Twist, Vector3
from geometry_msgs.msg import TransformStamped

from nav_msgs.msg import Odometry

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from turtle_brick_interfaces.msg import Tilt

from turtlesim.msg import Pose


def quaternion_from_euler(ai, aj, ak):
    """Convert to quaternion from euler angles."""
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q


def turtle_twist(linear_velocity, angular_velocity):
    """
    Create a twist velocity suitable for a turtle.

    Args:
    linear_vel (list of floats): the linear velocities
    angular_vel (list of floats): the angular velocities

    Returns
    -------
    Twist: a 2D twist object corresponding to linear/angular velocity

    """
    return Twist(
        linear=Vector3(
            x=linear_velocity[0], y=linear_velocity[1], z=linear_velocity[2]
        ),
        angular=Vector3(
            x=angular_velocity[0], y=angular_velocity[1], z=angular_velocity[2]
        ),
    )


def calculate_euclidean_distance(current_pos, goal_pos):
    """
    Calculate the distance between two points.

    Args:
    current_pos (turtlesim/Pose): the current position of the turtle
    current_waypoint (geometry_msgs/Point): the currently tracked
    waypoint (the waypoint to be visited)

    Returns
    -------
    tol - a double value corresponding to the distance
    between the current pose and current waypoint

    """
    tol = math.sqrt(
        ((current_pos.x) - goal_pos.pose.position.x) ** 2
        + ((current_pos.y) - goal_pos.pose.position.y) ** 2
    )

    return tol


class Turtle_Robot(Node):
    """The Turtle Robot Node."""

    def __init__(self):
        """Initialise the member variables of the class."""
        super().__init__('turtle_robot')

        # Publishers
        self.joint_state_publisher = self.create_publisher(
            JointState, 'joint_states', 1
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 1)

        # Timers
        self.turtle_tmr = self.create_timer(1 / 100, self.turtle_tmr_callback)
        self.joint_state_tmr = self.create_timer(
            1 / 100, self.joint_state_tmr_callback)
        self.cmd_vel_tmr = self.create_timer(
            1 / 250, self.cmd_vel_tmr_callback)

        # Subscribers
        self.turtlesim_pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.turtlesim_pose_callback, 1
        )
        self.goal_pose_subscriber = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_pose_callback, 1
        )
        self.tilt_angle_subscriber = self.create_subscription(
            Tilt, 'platform_tilt_angle', self.tilt_msg_callback, 1
        )

        # Static Transform Broadcaster world->odom
        self.static_world_broadcaster = StaticTransformBroadcaster(self)

        world_odom_tf = TransformStamped()
        world_odom_tf.header.stamp = self.get_clock().now().to_msg()
        world_odom_tf.header.frame_id = 'world'
        world_odom_tf.child_frame_id = 'odom'

        world_odom_tf.transform.translation.x = 5.54
        world_odom_tf.transform.translation.y = 5.54
        self.static_world_broadcaster.sendTransform(world_odom_tf)
        self.get_logger().info('Static Transfrom: world->odom')

        # Parameters
        self.declare_parameter('wheel_radius', 0.15)
        self.declare_parameter('max_velocity', 3.0)

        # Dynamic Transform Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Helper variables
        self.turtlesim_current_pose = Pose()
        self.turtlesim_current_pose.x = 5.54
        self.turtlesim_current_pose.y = 5.54

        self.current_velocity = Twist()

        self.goal_pose = PoseStamped()
        self.goal_pose.pose.position.x = 5.54
        self.goal_pose.pose.position.y = 5.54
        self.goal_pose.pose.position.z = 0.0

        self.wheel_joint_state = 0.0
        self.plat_tilt_angle = 0.0
        self.base_stem_angle = 0.0

    def turtle_tmr_callback(self):
        """Publish tf and odom msg at 100Hz."""
        wheel_radius = self.get_parameter('wheel_radius').value
        turtlesim_robot_tf = TransformStamped()
        turtlesim_robot_tf.header.stamp = self.get_clock().now().to_msg()
        turtlesim_robot_tf.header.frame_id = 'world'
        turtlesim_robot_tf.child_frame_id = 'base_link'
        turtlesim_x = self.turtlesim_current_pose.x
        turtlesim_y = self.turtlesim_current_pose.y
        turtlesim_robot_tf.transform.translation.x = turtlesim_x
        turtlesim_robot_tf.transform.translation.y = turtlesim_y
        turtlesim_robot_tf.transform.translation.z = wheel_radius * 2 + 0.2
        quat = quaternion_from_euler(0, 0, self.turtlesim_current_pose.theta)

        turtlesim_robot_tf.transform.rotation.x = quat[0]
        turtlesim_robot_tf.transform.rotation.y = quat[1]
        turtlesim_robot_tf.transform.rotation.z = quat[2]
        turtlesim_robot_tf.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(turtlesim_robot_tf)

        odom_msg = Odometry()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.twist.twist = self.current_velocity
        odom_msg.pose.pose.position.x = self.turtlesim_current_pose.x - 5.54
        odom_msg.pose.pose.position.y = self.turtlesim_current_pose.y - 5.54
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        self.odom_publisher.publish(odom_msg)

    def joint_state_tmr_callback(self):
        """Update the joint states at 100Hz."""
        wheel_radius = self.get_parameter('wheel_radius').value
        robot_joint_states = JointState()
        robot_joint_states.header.stamp = self.get_clock().now().to_msg()
        robot_joint_states.header.frame_id = 'odom'
        robot_joint_states.name = ['cylinder_platform',
                                   'base_stem', 'stem_wheel']

        forward_velocity = self.turtlesim_current_pose.linear_velocity

        self.wheel_joint_state += (forward_velocity)/(wheel_radius) * (1 / 100)

        robot_joint_states.position = [
            self.plat_tilt_angle,
            self.base_stem_angle,
            -self.wheel_joint_state,
        ]

        self.joint_state_publisher.publish(robot_joint_states)

    def cmd_vel_tmr_callback(self):
        """Command cmd_vel commands to the robot at 250Hz."""
        angular_diff = math.atan2(
            self.goal_pose.pose.position.y - (self.turtlesim_current_pose.y),
            self.goal_pose.pose.position.x - (self.turtlesim_current_pose.x),
        )

        self.base_stem_angle = angular_diff

        linear_velocity = 3.0
        x_vel = linear_velocity * math.cos(angular_diff)
        y_vel = linear_velocity * math.sin(angular_diff)

        cmd_twist = turtle_twist([x_vel, y_vel, 0.0], [0.0, 0.0, 0.0])

        if (
            calculate_euclidean_distance(self.turtlesim_current_pose,
                                         self.goal_pose)
            < 0.05
        ):
            cmd_twist = turtle_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        self.cmd_vel_publisher.publish(cmd_twist)
        self.current_velocity = cmd_twist

    def turtlesim_pose_callback(self, turtlesim_pose_msg):
        """
        Store the turtle's current pose.

        Args:
        ----
        turtlesim_pose_msg (turtlesim/Pose) : the turtle's current pose

        """
        self.turtlesim_current_pose = turtlesim_pose_msg

    def goal_pose_callback(self, goal_pose_msg):
        """
        Store the msg for the /goal_pose topic subscription.

        Args:
        ----
        goal_pose_msg (geometry_msg/PoseStamped) : the goal pose
        for the turtle

        """
        self.goal_pose = goal_pose_msg

    def tilt_msg_callback(self, tilt_msg):
        """
        Store the tilt angle for the platform.

        Args:
        ----
        tilt_msg (turtle_brick_interfaces/Tilt) : the tilt angle
        for the platform

        """
        self.plat_tilt_angle = tilt_msg.tilt_angle


def turtle_robot_start(args=None):
    """Spin the turtle node."""
    rclpy.init(args=args)
    node = Turtle_Robot()
    rclpy.spin(node)
    rclpy.shutdown()
