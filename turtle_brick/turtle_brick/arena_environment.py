import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import TransformStamped, Point
import tf2_ros
from tf2_ros import TransformBroadcaster
from turtle_brick_interfaces.srv import Place, Drop
from .physics import World 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from turtle_brick_interfaces.msg import Tilt
import math

class Arena(Node):
    """
    the node for setting up the arena
    """

    def __init__(self):
        super().__init__('arena')

        self.gravity_acceleration = self.declare_parameter('gravity_acceleration', 9.8) 
        self.max_vel = self.declare_parameter('max_velocity', 10.0)

        # Empty variable for brick_location
        self.current_brick_location = [2.0, 2.0, 5.0]
        self.platform_radius = 0.3
        # Setting up the physics module
        self.brick_physics = World([4.0, 4.0, 8.0], 9.8, self.platform_radius, 0.04)
    
        # Create service for placing the brick at a given position
        self.place_srv = self.create_service(Place, 'place', self.place_callback)

        # Creating service for dropping brick
        self.drop_srv = self.create_service(Drop, 'drop', self.drop_callback)  

        # Creating a client for the drop service
        self.drop_client = self.create_client(Drop, 'drop')

        # Creating a timer for the physics
        self.physics_tmr = self.create_timer((1/250), self.physics_tmr_callback)

        self.tf_buffer = Buffer()
        self.brick_platform_listener = TransformListener(self.tf_buffer, self)

        # create a marker publisher
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', markerQoS)

        self.brick_publisher = self.create_publisher(Marker, 'brick_marker', markerQoS)

        # markers for 4 walls of the arena
        self.m = Marker()
        self.m.header.frame_id = "odom"
        self.m.header.stamp = self.get_clock().now().to_msg()
        self.m.id = 1
        self.m.type = Marker.CUBE
        self.m.action = Marker.ADD
        self.m.scale.x = 11.0
        self.m.scale.y = 0.5
        self.m.scale.z = 3.0
        self.m.pose.position.x = 0.0
        self.m.pose.position.y = -5.56
        self.m.pose.position.z = 1.5
        self.m.pose.orientation.x = 0.0
        self.m.pose.orientation.y = 0.0
        self.m.pose.orientation.z = 0.0
        self.m.pose.orientation.w = 1.0
        self.m.color.r = 0.0
        self.m.color.g = 0.0
        self.m.color.b = 1.0
        self.m.color.a = 1.0
        self.marker_publisher.publish(self.m)

        self.m1 = Marker()
        self.m1.header.frame_id = "odom"
        self.m1.header.stamp = self.get_clock().now().to_msg()
        self.m1.id = 2
        self.m1.type = Marker.CUBE
        self.m1.action = Marker.ADD
        self.m1.scale.x = 11.0
        self.m1.scale.y = 0.5
        self.m1.scale.z = 3.0
        self.m1.pose.position.x = 0.0
        self.m1.pose.position.y = 5.56
        self.m1.pose.position.z = 1.5
        self.m1.pose.orientation.x = 0.0
        self.m1.pose.orientation.y = 0.0
        self.m1.pose.orientation.z = 0.0
        self.m1.pose.orientation.w = 1.0
        self.m1.color.r = 0.0
        self.m1.color.g = 0.0
        self.m1.color.b = 1.0
        self.m1.color.a = 1.0
        self.marker_publisher.publish(self.m1)


        self.m2 = Marker()
        self.m2.header.frame_id = "odom"
        self.m2.header.stamp = self.get_clock().now().to_msg()
        self.m2.id = 3
        self.m2.type = Marker.CUBE
        self.m2.action = Marker.ADD
        self.m2.scale.x = 11.0
        self.m2.scale.y = 0.5
        self.m2.scale.z = 3.0
        self.m2.pose.position.x = 5.56
        self.m2.pose.position.y = 0.0
        self.m2.pose.position.z = 1.5
        self.m2.pose.orientation.x = 0.0
        self.m2.pose.orientation.y = 0.0
        self.m2.pose.orientation.z = 0.707
        self.m2.pose.orientation.w = 0.707
        self.m2.color.r = 0.0
        self.m2.color.g = 0.0
        self.m2.color.b = 1.0
        self.m2.color.a = 1.0
        self.marker_publisher.publish(self.m2)

        self.m3 = Marker()
        self.m3.header.frame_id = "odom"
        self.m3.header.stamp = self.get_clock().now().to_msg()
        self.m3.id = 4
        self.m3.type = Marker.CUBE
        self.m3.action = Marker.ADD
        self.m3.scale.x = 11.0
        self.m3.scale.y = 0.5
        self.m3.scale.z = 3.0
        self.m3.pose.position.x = -5.56
        self.m3.pose.position.y = 0.0
        self.m3.pose.position.z = 1.5
        self.m3.pose.orientation.x = 0.0
        self.m3.pose.orientation.y = 0.0
        self.m3.pose.orientation.z = 0.707
        self.m3.pose.orientation.w = 0.707
        self.m3.color.r = 0.0
        self.m3.color.g = 0.0
        self.m3.color.b = 1.0
        self.m3.color.a = 1.0
        self.marker_publisher.publish(self.m3)

        
        self.brick_tf_broadcaster = TransformBroadcaster(self)

        self.tilt_angle = 0.0

        self.tilt_angle_subscriber = self.create_subscription(
            Tilt, "platform_tilt_angle", self.tilt_msg_callback, 1
        )

           
        self.flag = False
        self.move_brick = False
        self.move_brick_x = True
        self.move_brick_z = True
        self.drop_client.call_async(Drop.Request())

    def physics_tmr_callback(self):
        self.current_brick_location = self.brick_physics.brick

        odom_brick_tf = TransformStamped()
        odom_brick_tf.header.frame_id = 'odom'
        odom_brick_tf.child_frame_id = 'brick'
        odom_brick_tf.header.stamp = self.get_clock().now().to_msg()
        # brick_location = self.brick_physics.brick
        odom_brick_tf.transform.translation.x = self.current_brick_location[0]
        odom_brick_tf.transform.translation.y = self.current_brick_location[1]
        odom_brick_tf.transform.translation.z = self.current_brick_location[2]

        try:
            odom_platform_lookup = self.tf_buffer.lookup_transform('odom', 'platform', rclpy.time.Time())
            odom_brick_tf.transform.rotation = odom_platform_lookup.transform.rotation
        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().info(f'Lookup exception: {e}')
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f'Connectivity exception: {e}')
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().info(f'Extrapolation exception: {e}') 

       
        self.brick_tf_broadcaster.sendTransform(odom_brick_tf)

        self.brick = Marker()
        self.brick.header.frame_id = "odom"
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
        self.brick.pose.orientation.x = odom_brick_tf.transform.rotation.x
        self.brick.pose.orientation.y = odom_brick_tf.transform.rotation.y
        self.brick.pose.orientation.z = odom_brick_tf.transform.rotation.z
        self.brick.pose.orientation.w = odom_brick_tf.transform.rotation.w
        self.brick.color.r = 1.0
        self.brick.color.g = 0.0
        self.brick.color.b = 0.0
        self.brick.color.a = 1.0
        self.brick_publisher.publish(self.brick) 



        try:
            brick_platform = self.tf_buffer.lookup_transform('brick', 'platform', rclpy.time.Time())
            threshold_brick_platform = (brick_platform.transform.translation.x**2 + brick_platform.transform.translation.y**2 + brick_platform.transform.translation.z**2)**0.5
            if(threshold_brick_platform <= 0.15):
                self.flag = False
                self.move_brick = True
        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().info(f'Lookup exception: {e}')
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f'Connectivity exception: {e}')
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().info(f'Extrapolation exception: {e}')    

        try:
            odom_brick_lookup = self.tf_buffer.lookup_transform('odom', 'brick', rclpy.time.Time())
            threshold_odom_brick = odom_brick_lookup.transform.translation.z
            if(threshold_odom_brick <= 0.1):
                self.flag = False
        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().info(f'Lookup exception: {e}')
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f'Connecticvity exception: {e}')
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().info(f'Extrapolation exception: {e}') 



        if(self.flag):
            self.brick_physics.drop()
        elif(self.move_brick):
            try:
                odom_platform_lookup = self.tf_buffer.lookup_transform('odom', 'platform', rclpy.time.Time())
                if(odom_platform_lookup.transform.rotation.x != 0.0 or odom_platform_lookup.transform.rotation.y !=0.0):
                    if(self.move_brick_x):
                        self.brick_physics.drop_brick_x(self.tilt_angle)
                    if(self.move_brick_z):
                        self.brick_physics.drop_brick_z(self.tilt_angle)
                    odom_brick_lookup = self.tf_buffer.lookup_transform('odom', 'brick', rclpy.time.Time())
                    z_displacement = odom_platform_lookup.transform.translation.z - odom_brick_lookup.transform.translation.z
                    x_displacement = odom_platform_lookup.transform.translation.x - odom_brick_lookup.transform.translation.x
                    # if(platform_brick_lookup.transform.translation.z  <= (0.125 - ((self.platform_radius + 0.075 )*math.sin(self.tilt_angle))) or platform_brick_lookup.transform.translation.z  >= -(0.125 - ((self.platform_radius + 0.075 )*math.sin(self.tilt_angle)) )):
                    if(z_displacement > (0.400* math.sin(self.tilt_angle))):
                        self.move_brick_z = False
                    if((x_displacement) > (0.400* math.cos(self.tilt_angle))):
                        self.move_brick_x = False
                    if(self.move_brick_x == False and self.move_brick_z == False):
                        self.move_brick = False
                else:
                    platform_location = Point()
                    platform_location.x = odom_platform_lookup.transform.translation.x
                    platform_location.y = odom_platform_lookup.transform.translation.y
                    platform_location.z = odom_platform_lookup.transform.translation.z + 0.05 + 0.075
                    self.brick_physics.brick = platform_location
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
            
        # elif(self.flag == 2):
        #     pass

        
        
    def place_callback(self, request, response):
        print(request.brick_position) 
        self.brick_physics.brick= request.brick_position
        return response
    
    def drop_callback(self, request, response):
        self.flag = True
        return response
    
    def tilt_msg_callback(self, tilt_msg):
        self.tilt_angle = tilt_msg.tilt_angle


def start_arena(args=None):
    rclpy.init(args=args)
    node = Arena()
    rclpy.spin(node)
    rclpy.shutdown()
