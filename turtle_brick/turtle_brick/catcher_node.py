"""The Catcher Node."""
from builtin_interfaces.msg import Duration

from geometry_msgs.msg import Point, PoseStamped

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtle_brick_interfaces.msg import Tilt

from turtlesim.msg import Pose

from visualization_msgs.msg import Marker


class Catcher(Node):
    """Node to catch the brick."""

    def __init__(self):
        """Initialise the member variables."""
        super().__init__('catcher')

        self.max_vel = self.declare_parameter('max_velocity', 3.0)
        self.platform_height = self.declare_parameter('platform_height', 0.8)
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.15)
      

        self.tmr_to_brick = self.create_timer(1/250,
                                              self.tmr_to_brick_callback)

        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer, self)

        self.drop_status_subscriber = self.create_subscription(Point, 'drop_status', self.drop_status_callback, 1)

        self.turtlesim_pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.turtlesim_pose_callback, 1
        )

        self.turtlesim_current_pose = Pose()
        self.turtlesim_current_pose.x = 5.54
        self.turtlesim_current_pose.y = 5.54

        self.goal_pose_publisher = self.create_publisher(
            PoseStamped, 'goal_pose', 10)

        markerQoS = QoSProfile(depth=10,
                               durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_publisher = self.create_publisher(
            Marker, 'text_marker', markerQoS)

        self.tilt_angle_publisher = self.create_publisher(Tilt, 'platform_tilt_angle', 10)
        
  

        self.display_flag = True
        self.brick_drop_status = False
        self.iter = 0

    def tmr_to_brick_callback(self):
        """Decide and command the robot to catch the brick at 250Hz."""
        plat_height = self.get_parameter('platform_height').value
        wheel_radius = self.get_parameter('wheel_radius').value
        max_velocity = self.get_parameter('max_velocity').value
        try:
            world_brick_lookup = self.tf_buffer.lookup_transform(
                'world', 'brick', rclpy.time.Time())
            current_brick_height = world_brick_lookup.transform.translation.z
            brick_x = world_brick_lookup.transform.translation.x
            brick_y = world_brick_lookup.transform.translation.y
     

            if (self.brick_drop_status is True):
                    
                    dist_to_platform = (current_brick_height
                                        - (plat_height +
                                           (wheel_radius*2) + 0.2))

                    x_dist = (self.turtlesim_current_pose.x) - (
                        brick_x
                        )
                    y_dist = (self.turtlesim_current_pose.y) - (
                        brick_y
                        )

                    planar_dist_to_brick = ((x_dist)**2
                                            + (y_dist)**2)**0.5

                    minimum_time_to_brick = planar_dist_to_brick/max_velocity

                    if (dist_to_platform > 0.0):
                        time_to_platform = ((dist_to_platform * 2)/9.8)**0.5
                        if (time_to_platform < minimum_time_to_brick):

                            if(self.display_flag is True):
                                self.display_msg_marker()
                                self.get_logger().info('Cant reach the brick')
                                self.display_flag = False
                        else:
                            if(self.display_flag is True):
                                self.get_logger().info("I can reach the brick!")
                                self.display_flag = False
                            goal_pose = PoseStamped()
                            goal_pose.pose.position.x = (
                                world_brick_lookup.transform.translation.x
                                )
                            goal_pose.pose.position.y = (
                                world_brick_lookup.transform.translation.y
                                )
                            self.goal_pose_publisher.publish(goal_pose)
                            self.brick_drop_status = False
                            
            try:
                brick_platform_lookup = self.tf_buffer.lookup_transform(
                    'brick', 'platform', rclpy.time.Time())

                if (
                    brick_platform_lookup.transform.translation.x == 0.0):
                    center_pose = PoseStamped()
                    center_pose.pose.position.x = 5.54
                    center_pose.pose.position.y = 5.54
                    self.goal_pose_publisher.publish(center_pose)
                    self.brick_drop_status = False
                    x_disp = abs(self.turtlesim_current_pose.x - center_pose.pose.position.x)
                    y_disp = abs(self.turtlesim_current_pose.y - center_pose.pose.position.y)
                    if(x_disp < 0.05 and y_disp < 0.05):
                        tilt_cmd = Tilt()
                        tilt_cmd.tilt_angle = 0.707
                        self.tilt_angle_publisher.publish(tilt_cmd)

            except tf2_ros.LookupException as e:
                # the frames don't exist yet
                self.get_logger().debug(f'Lookup exception: {e}')
            except tf2_ros.ConnectivityException as e:
                # the tf tree has a disconnection
                self.get_logger().debug(f'Connectivity exception: {e}')
            except tf2_ros.ExtrapolationException as e:
                # the times are two far apart to extrapolate
                self.get_logger().debug(f'Extrapolation exception: {e}')

        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().debug(f'Lookup exception: {e}')
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().debug(f'Connecticvity exception: {e}')
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().debug(f'Extrapolation exception: {e}')

        


    def display_msg_marker(self):
        """Initialise the text marker."""
        self.m = Marker()
        self.m.header.frame_id = 'world'
        self.m.header.stamp = self.get_clock().now().to_msg()
        self.m.id = 6
        self.m.type = Marker.TEXT_VIEW_FACING
        self.m.action = Marker.ADD
        self.m.scale.x = 1.0
        self.m.scale.y = 1.0
        self.m.scale.z = 1.0
        self.m.pose.position.x = 5.54
        self.m.pose.position.y = 5.54
        self.m.pose.position.z = 1.5
        self.m.pose.orientation.x = 0.0
        self.m.pose.orientation.y = 0.0
        self.m.pose.orientation.z = 0.0
        self.m.pose.orientation.w = 1.0
        self.m.color.r = 1.0
        self.m.color.g = 0.0
        self.m.color.b = 1.0
        self.m.color.a = 1.0

        marker_duration = Duration()
        marker_duration.sec = 3

        self.m.lifetime = marker_duration
        self.m.text = 'Unreachable'
        self.marker_publisher.publish(self.m)

    def turtlesim_pose_callback(self, turtlesim_pose_msg):
        """
        Store the turtle's pose.

        Args:
        ----
        turtlesim_pose_msg (turtlesim/Pose) : the turtle's current pose

        """
        self.turtlesim_current_pose = turtlesim_pose_msg

    def drop_status_callback(self, drop_status_msg):
        """
        Check the status of the brick.

        Args:
        ----
        drop_status_msg (geometry_msgs/Point) : the trigger for drop status
        """
        if(drop_status_msg.x == 1.0):
            self.brick_drop_status = True
            self.display_flag = True

def catch_brick(args=None):
    """Spin the Catcher Node."""
    rclpy.init(args=args)
    node = Catcher()
    rclpy.spin(node)
    rclpy.shutdown()
