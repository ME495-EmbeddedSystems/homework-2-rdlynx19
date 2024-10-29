"""The Arena Node."""
from geometry_msgs.msg import Point, TransformStamped

import rclpy
from rclpy.node import Node
import rclpy.parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
import rclpy.time

import tf2_ros
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtle_brick_interfaces.msg import Tilt
from turtle_brick_interfaces.srv import Drop, Place

from visualization_msgs.msg import Marker

from .physics import World


class Arena(Node):
    """The Arena Node."""

    def __init__(self):
        """Initialise the member variables for the arena."""
        super().__init__('arena')

        # QoS Profile
        markerQoS = QoSProfile(depth=10,
                               durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # Publishers
        self.marker_publisher = self.create_publisher(
            Marker, 'visualization_marker', markerQoS)
        self.brick_publisher = self.create_publisher(Marker,
                                                     'brick_marker', markerQoS)
        self.drop_comms_publisher = self.create_publisher(
            Point, 'drop_status', 10)
        self.tilt_angle_publisher = self.create_publisher(
            Tilt, 'platform_tilt_angle', 10)

        # Subscriber
        self.tilt_angle_subscriber = self.create_subscription(
            Tilt, 'platform_tilt_angle', self.tilt_msg_callback, 1
        )
        # Parameters
        self.gravity_acceleration = self.declare_parameter('gravity_accel',
                                                           9.8)
        # gravity_acceleration = self.get_parameter("wheel_radius").value
        self.max_velocity = self.declare_parameter('max_velocity', 3.0)

        # Services
        self.drop_srv = self.create_service(Drop, 'drop', self.drop_callback)
        self.place_srv = self.create_service(Place,
                                             'place', self.place_callback)

        # Timers
        self.physics_tmr = self.create_timer((1/250),
                                             self.physics_tmr_callback)
        # Clients
        self.drop_client = self.create_client(Drop, 'drop')

        # Buffer for Transform Lookup
        self.tf_buffer = Buffer()
        self.brick_platform_listener = TransformListener(self.tf_buffer, self)

        # Initialising the physics World
        self.current_brick_location = [6.4, 6.4, 8.0]
        self.platform_radius = 0.3
        self.brick_physics = World([self.current_brick_location[0],
                                    self.current_brick_location[1],
                                    self.current_brick_location[2]],
                                   9.8, self.platform_radius, 0.004)

        # Initialising and publishing the wall markers
        self.m = Marker()
        self.m.header.frame_id = 'world'
        self.m.header.stamp = self.get_clock().now().to_msg()
        self.m.id = 1
        self.m.type = Marker.CUBE
        self.m.action = Marker.ADD
        self.m.scale.x = 11.0
        self.m.scale.y = 0.5
        self.m.scale.z = 3.0
        self.m.pose.position.x = 5.54
        self.m.pose.position.y = 0.0
        self.m.pose.position.z = 1.5
        self.m.pose.orientation.x = 0.0
        self.m.pose.orientation.y = 0.0
        self.m.pose.orientation.z = 0.0
        self.m.pose.orientation.w = 1.0
        self.m.color.r = 0.305
        self.m.color.g = 0.164
        self.m.color.b = 0.517
        self.m.color.a = 1.0
        self.marker_publisher.publish(self.m)

        self.m1 = Marker()
        self.m1.header.frame_id = 'world'
        self.m1.header.stamp = self.get_clock().now().to_msg()
        self.m1.id = 2
        self.m1.type = Marker.CUBE
        self.m1.action = Marker.ADD
        self.m1.scale.x = 11.0
        self.m1.scale.y = 0.5
        self.m1.scale.z = 3.0
        self.m1.pose.position.x = 5.54
        self.m1.pose.position.y = 11.0
        self.m1.pose.position.z = 1.5
        self.m1.pose.orientation.x = 0.0
        self.m1.pose.orientation.y = 0.0
        self.m1.pose.orientation.z = 0.0
        self.m1.pose.orientation.w = 1.0
        self.m1.color.r = 0.305
        self.m1.color.g = 0.164
        self.m1.color.b = 0.517
        self.m1.color.a = 1.0
        self.marker_publisher.publish(self.m1)

        self.m2 = Marker()
        self.m2.header.frame_id = 'world'
        self.m2.header.stamp = self.get_clock().now().to_msg()
        self.m2.id = 3
        self.m2.type = Marker.CUBE
        self.m2.action = Marker.ADD
        self.m2.scale.x = 11.0
        self.m2.scale.y = 0.5
        self.m2.scale.z = 3.0
        self.m2.pose.position.x = 11.0
        self.m2.pose.position.y = 5.54
        self.m2.pose.position.z = 1.5
        self.m2.pose.orientation.x = 0.0
        self.m2.pose.orientation.y = 0.0
        self.m2.pose.orientation.z = 0.707
        self.m2.pose.orientation.w = 0.707
        self.m2.color.r = 0.305
        self.m2.color.g = 0.164
        self.m2.color.b = 0.517
        self.m2.color.a = 1.0
        self.marker_publisher.publish(self.m2)

        self.m3 = Marker()
        self.m3.header.frame_id = 'world'
        self.m3.header.stamp = self.get_clock().now().to_msg()
        self.m3.id = 4
        self.m3.type = Marker.CUBE
        self.m3.action = Marker.ADD
        self.m3.scale.x = 11.0
        self.m3.scale.y = 0.5
        self.m3.scale.z = 3.0
        self.m3.pose.position.x = 0.0
        self.m3.pose.position.y = 5.54
        self.m3.pose.position.z = 1.5
        self.m3.pose.orientation.x = 0.0
        self.m3.pose.orientation.y = 0.0
        self.m3.pose.orientation.z = 0.707
        self.m3.pose.orientation.w = 0.707
        self.m3.color.r = 0.305
        self.m3.color.g = 0.164
        self.m3.color.b = 0.517
        self.m3.color.a = 1.0
        self.marker_publisher.publish(self.m3)

        # Dynamic Transform Broadcaster
        self.brick_tf_broadcaster = TransformBroadcaster(self)

        # Helper variables
        self.flag = False
        self.move_brick = False
        self.move_brick_x = True
        self.move_brick_z = False
        self.change_brick_parent = False
        self.tilt_angle = 0.0


    def physics_tmr_callback(self):
        """Compute the physics and transforms at 250Hz."""
        self.current_brick_location = self.brick_physics.brick

        if (self.change_brick_parent is False):
            world_brick_tf = TransformStamped()
            world_brick_tf.header.frame_id = 'world'
            world_brick_tf.child_frame_id = 'brick'
            world_brick_tf.header.stamp = self.get_clock().now().to_msg()
            world_brick_tf.transform.translation.x = (
                self.current_brick_location[0])
            world_brick_tf.transform.translation.y = (
                self.current_brick_location[1])
            world_brick_tf.transform.translation.z = (
                self.current_brick_location[2])

            try:
                world_platform_lookup = self.tf_buffer.lookup_transform(
                    'world', 'platform', rclpy.time.Time())
                platform_rotation = world_platform_lookup.transform.rotation
                world_brick_tf.transform.rotation = platform_rotation
            except tf2_ros.LookupException as e:
                # the frames don't exist yet
                self.get_logger().debug(f'Lookup exception: {e}')
            except tf2_ros.ConnectivityException as e:
                # the tf tree has a disconnection
                self.get_logger().debug(f'Connectivity exception: {e}')
            except tf2_ros.ExtrapolationException as e:
                # the times are two far apart to extrapolate
                self.get_logger().debug(f'Extrapolation exception: {e}')

            self.brick_tf_broadcaster.sendTransform(world_brick_tf)

            self.brick = Marker()
            self.brick.header.frame_id = 'world'
            self.brick.header.stamp = self.get_clock().now().to_msg()
            self.brick.id = 5
            self.brick.type = Marker.CUBE
            self.brick.action = Marker.ADD
            self.brick.scale.x = 0.3
            self.brick.scale.y = 0.15
            self.brick.scale.z = 0.15
            self.brick.pose.position.x = self.current_brick_location[0]
            self.brick.pose.position.y = self.current_brick_location[1]
            self.brick.pose.position.z = self.current_brick_location[2]
            self.brick.pose.orientation.x = world_brick_tf.transform.rotation.x
            self.brick.pose.orientation.y = world_brick_tf.transform.rotation.y
            self.brick.pose.orientation.z = world_brick_tf.transform.rotation.z
            self.brick.pose.orientation.w = world_brick_tf.transform.rotation.w
            self.brick.color.r = 1.0
            self.brick.color.g = 0.0
            self.brick.color.b = 0.0
            self.brick.color.a = 1.0
            self.brick_publisher.publish(self.brick)

        elif (self.change_brick_parent is True):
            platform_brick_tf = TransformStamped()
            platform_brick_tf.header.frame_id = 'platform'
            platform_brick_tf.child_frame_id = 'brick'
            platform_brick_tf.header.stamp = self.get_clock().now().to_msg()
            platform_brick_tf.transform.translation.x = (
                self.current_brick_location[0])
            platform_brick_tf.transform.translation.y = (
                self.current_brick_location[1])
            platform_brick_tf.transform.translation.z = (
                self.current_brick_location[2])

            self.brick_tf_broadcaster.sendTransform(platform_brick_tf)

            self.brick = Marker()
            self.brick.header.frame_id = 'platform'
            self.brick.header.stamp = self.get_clock().now().to_msg()
            self.brick.id = 5
            self.brick.type = Marker.CUBE
            self.brick.action = Marker.ADD
            self.brick.scale.x = 0.3
            self.brick.scale.y = 0.15
            self.brick.scale.z = 0.15
            self.brick.pose.position.x = self.current_brick_location[0]
            self.brick.pose.position.y = self.current_brick_location[1]
            self.brick.pose.position.z = self.current_brick_location[2]
            self.brick.pose.orientation.x = (
                platform_brick_tf.transform.rotation.x)
            self.brick.pose.orientation.y = (
                platform_brick_tf.transform.rotation.y)
            self.brick.pose.orientation.z = (
                platform_brick_tf.transform.rotation.z)
            self.brick.pose.orientation.w = (
                platform_brick_tf.transform.rotation.w)
            self.brick.color.r = 1.0
            self.brick.color.g = 0.0
            self.brick.color.b = 0.0
            self.brick.color.a = 1.0
            self.brick_publisher.publish(self.brick)

        try:
            brick_platform = self.tf_buffer.lookup_transform(
                'brick', 'platform', rclpy.time.Time())
            x_translation = brick_platform.transform.translation.x
            y_translation = brick_platform.transform.translation.y
            z_translation = brick_platform.transform.translation.z
            threshold_brick_platform = (
                x_translation**2 + y_translation**2 + z_translation**2)**0.5
            if (threshold_brick_platform <= 0.25):
                self.flag = False
                self.move_brick = True
                self.change_brick_parent = True
                self.brick_physics.brick_caught()
        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().debug(f'Lookup exception: {e}')
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().debug(f'Connectivity exception: {e}')
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().debug(f'Extrapolation exception: {e}')

        try:
            odom_brick_lookup = self.tf_buffer.lookup_transform(
                'odom', 'brick', rclpy.time.Time())
            threshold_odom_brick = odom_brick_lookup.transform.translation.z
            if (threshold_odom_brick <= 0.1):
                self.flag = False
        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().debug(f'Lookup exception: {e}')
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().debug(f'Connecticvity exception: {e}')
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().debug(f'Extrapolation exception: {e}')

        if (self.flag is True):
            self.brick_physics.drop()
        elif (self.move_brick is True):
            try:

                world_platform_lookup = (
                    self.tf_buffer.lookup_transform('world', 'platform',
                                                    rclpy.time.Time()))

                if (world_platform_lookup.transform.rotation.x != 0.0
                        or world_platform_lookup.transform.rotation.y != 0.0):
                    if (self.move_brick_x):
                        self.brick_physics.drop_brick_x(self.tilt_angle)

                    brick_platform_lookup = (
                        self.tf_buffer.lookup_transform('brick',
                                                        'platform',
                                                        rclpy.time.Time()))

                    if (
                        (brick_platform_lookup.transform.translation.x >
                         (0.450)) or
                        (brick_platform_lookup.transform.translation.x <
                         (-0.450))):
                        self.move_brick_x = False
                        self.move_brick = False
                        self.current_brick_location = self.brick_physics.brick
                else:
                    if (self.change_brick_parent is False):
                        platform_location = Point()
                        platform_location.x = (
                            world_platform_lookup.transform.translation.x)
                        platform_location.y = (
                            world_platform_lookup.transform.translation.y)
                        platform_location.z = (
                            world_platform_lookup.transform.translation.z
                            + 0.05 + 0.075)
                        self.brick_physics.brick = platform_location
                        self.current_brick_location = self.brick_physics.brick

                    else:
                        # self.get_logger().info("Brick changing!")
                        brick_location = Point()
                        brick_location.x = 0.0
                        brick_location.y = 0.0
                        brick_location.z = 0.05 + 0.075
                        self.brick_physics.brick = brick_location
                        self.current_brick_location = self.brick_physics.brick
            except tf2_ros.LookupException as e:
                # the frames don't exist yet
                self.get_logger().info(f'Lookup exception: {e}')
            except tf2_ros.ConnectivityException as e:
                # the tf tree has a disconnection
                self.get_logger().info(f'Connectivity exception: {e}')
            except tf2_ros.ExtrapolationException as e:
                # the times are two far apart to extrapolate
                self.get_logger().info(f'Extrapolation exception: {e}')

    def place_callback(self, request, response):
        """
        Place the brick at a position.

        Args:
        request(turtle_brick_interfaces/Place) : place request object
        response(EmptyResponse) : empty response object

        Returns
        -------
        An empty response object

        """
        self.brick_physics.brick = request.brick_position
        self.current_brick_location = self.brick_physics.brick
        self.flag = False
        self.move_brick = False
        self.change_brick_parent = False
        resetTilt = Tilt()
        resetTilt.tilt_angle = 0.0
        self.tilt_angle_publisher.publish(resetTilt)
        return response

    def drop_callback(self, request, response):
        """
        Start dropping the brick in gravity.

        Args:
        request(turtle_brick_interfaces/Drop) : drop request object
        response(bool) : bool response

        Returns
        -------
        A bool response

        """
        self.flag = True
        drop_trigger = Point()
        drop_trigger.x = 1.0
        self.drop_comms_publisher.publish(drop_trigger)
        return response

    def tilt_msg_callback(self, tilt_msg):
        """
        Store the tilt angle of the platform.

        Args:
        ----
        tilt_msg(turtle_brick_interfaces/Tilt) : the tilt angle for
        the platform

        """
        self.tilt_angle = tilt_msg.tilt_angle


def start_arena(args=None):
    """Spin the arena node."""
    rclpy.init(args=args)
    node = Arena()
    rclpy.spin(node)
    rclpy.shutdown()
