import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist, Vector3, PoseStamped

import math
import numpy as np
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from sensor_msgs.msg import JointState
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from turtle_brick_interfaces.msg import Tilt



def quaternion_from_euler(ai, aj, ak):
    """
    Convert to quaternion from euler angles
    """
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


def calculate_euclidean_distance(current_pos, goal_pos):
    """Calculate the distance between the current position and currently tracked waypoint

    Args:
        current_pos (turtlesim/Pose): the current position of the turtle
        current_waypoint (geometry_msgs/Point): the currently tracked waypoint (the waypoint to be visited)

    Returns:
        tol - a double value corresponding to the distance between the current pose and current waypoint

    """
    tol = math.sqrt(
        ((current_pos.x) - goal_pos.pose.position.x) ** 2
        + ((current_pos.y) - goal_pos.pose.position.y) ** 2
    )

    return tol


class Turtle_Robot(Node):
    """
    Turtle Robot Node
    """

    def __init__(self):
        super().__init__("turtle_robot")

        # transformation from odom to base_link
        self.tf_broadcaster = TransformBroadcaster(self)
        self.turtle_tmr = self.create_timer(1 / 100, self.turtle_tmr_callback)

        # joint state publisher
        self.joint_state_publisher = self.create_publisher(
            JointState, "joint_states", 1
        )
        self.joint_state_tmr = self.create_timer(1 / 100, self.joint_state_tmr_callback)

        # turtle pose subscriber
        self.turtlesim_current_pose = (
            Pose()
        )  # class variable to store current pose of the turtlesim
        self.turtlesim_current_pose.x = 0.0
        self.turtlesim_current_pose.y = 0.0
        self.turtlesim_pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.turtlesim_pose_callback, 1
        )

        # odom message publisher
        self.odom_publisher = self.create_publisher(Odometry, "odom", 1)

        # cmd_vel publisher
        self.current_velocity = Twist()  # creating a twist class variable
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 1)
        self.cmd_vel_tmr = self.create_timer(1 / 250, self.cmd_vel_tmr_callback)

        # goal pose subscriber
        self.goal_pose = PoseStamped()
        self.goal_pose.pose.position.x = 5.54
        self.goal_pose.pose.position.y = 5.54
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose_subscriber = self.create_subscription(
            PoseStamped, "goal_pose", self.goal_pose_callback, 1
        )

        # static broadcaster for the world frame
        self.static_world_broadcaster = StaticTransformBroadcaster(self)

        world_odom_tf = TransformStamped()
        world_odom_tf.header.stamp = self.get_clock().now().to_msg()
        world_odom_tf.header.frame_id = "world"
        world_odom_tf.child_frame_id = "odom"

        world_odom_tf.transform.translation.x = 5.54
        world_odom_tf.transform.translation.y = 5.54
        self.static_world_broadcaster.sendTransform(world_odom_tf)
        self.get_logger().info("Static Transfrom: world->odom")

        # tilt message subscriber
        self.tilt_angle_subscriber = self.create_subscription(
            Tilt, "platform_tilt_angle", self.tilt_msg_callback, 1
        )

        # creating parameters for these useful properties like wheel_radius
        self.declare_parameter('wheel_radius', 0.15)
        self.wheel_joint_state = 0.0
        self.plat_tilt_angle = 0.0
        self.base_stem_angle = 0.0

    def turtle_tmr_callback(self):
        wheel_radius = self.get_parameter('wheel_radius').value
        turtlesim_robot_tf = TransformStamped()
        turtlesim_robot_tf.header.stamp = self.get_clock().now().to_msg()
        turtlesim_robot_tf.header.frame_id = "odom"
        turtlesim_robot_tf.child_frame_id = "base_link"
        turtlesim_robot_tf.transform.translation.x = (
            self.turtlesim_current_pose.x - 5.54
        )
        turtlesim_robot_tf.transform.translation.y = (
            self.turtlesim_current_pose.y - 5.54
        )
        turtlesim_robot_tf.transform.translation.z = wheel_radius*2 + 0.2
        quat = quaternion_from_euler(0, 0, self.turtlesim_current_pose.theta)

        turtlesim_robot_tf.transform.rotation.x = quat[0]
        turtlesim_robot_tf.transform.rotation.y = quat[1]
        turtlesim_robot_tf.transform.rotation.z = quat[2]
        turtlesim_robot_tf.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(turtlesim_robot_tf)

        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
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
        wheel_radius = self.get_parameter('wheel_radius').value
        robot_joint_states = JointState()
        robot_joint_states.header.stamp = self.get_clock().now().to_msg()
        robot_joint_states.header.frame_id = "odom"
        robot_joint_states.name = ["cylinder_platform", "base_stem", "stem_wheel"]

        forward_velocity = self.turtlesim_current_pose.linear_velocity
        # forward_velocity = math.sqrt(self.current_velocity.linear.x **2 + self.current_velocity.linear.y**2)
        self.wheel_joint_state += (forward_velocity) / (wheel_radius) * (1 / 100)
        # if (self.wheel_joint_state > 3.14):
            # self.wheel_joint_state = self.wheel_joint_state - 3.14


        
        robot_joint_states.position = [self.plat_tilt_angle, self.base_stem_angle, -self.wheel_joint_state]

        self.joint_state_publisher.publish(robot_joint_states)

    def cmd_vel_tmr_callback(self):
        # publish cmd_vel messages here
        # use goal pose to calculate apt cmd_vel messages
        angular_diff = math.atan2(
            self.goal_pose.pose.position.y - (self.turtlesim_current_pose.y),
            self.goal_pose.pose.position.x - (self.turtlesim_current_pose.x),
        )

        self.base_stem_angle = angular_diff

        # yaw_vel = 0.1 * (angular_diff - self.turtlesim_current_pose.theta) + 0.05 * (
        #     angular_diff - self.turtlesim_current_pose.theta
        # ) / (1 / 100)
        x_vel = 4.0 * calculate_euclidean_distance(
            self.turtlesim_current_pose, self.goal_pose
        )

        lin_vel = min(5.0, 4.0 * calculate_euclidean_distance(self.turtlesim_current_pose, self.goal_pose))

        x_vel = lin_vel*math.cos(angular_diff)
        y_vel = lin_vel*math.sin(angular_diff)


        # if(x_vel > 5.0):
            # x_vel = 5.0
        cmd_twist = turtle_twist([x_vel, y_vel, 0.0], [0.0, 0.0, 0.0])
        
        # turtlesim_orientation = quaternion_from_euler(0.0, 0.0, self.turtlesim_current_pose.theta)

        if(calculate_euclidean_distance(self.turtlesim_current_pose, self.goal_pose) < 0.05 ):
            cmd_twist = turtle_twist([0.0, 0.0, 0.0],[0.0, 0.0, 0.0])
        self.cmd_vel_publisher.publish(cmd_twist)
        self.current_velocity = cmd_twist

    def turtlesim_pose_callback(self, turtlesim_pose_msg):
        # if(turtlesim_pose_msg.theta > 3.06 or turtlesim_pose_msg.theta < -3.06):
        #     turtlesim_pose_msg.theta = 0.0
        self.turtlesim_current_pose = turtlesim_pose_msg
        

    def goal_pose_callback(self, goal_pose_msg):
        # command velocity to send robot to goal pose
        self.goal_pose = goal_pose_msg
        # pass

    def tilt_msg_callback(self, tilt_msg):
        self.plat_tilt_angle = tilt_msg.tilt_angle
        


def turtle_robot_start(args=None):
    rclpy.init(args=args)
    node = Turtle_Robot()
    rclpy.spin(node)
    rclpy.shutdown()
