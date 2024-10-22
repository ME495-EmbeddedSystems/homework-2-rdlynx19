import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from turtle_brick_interfaces.srv import Place
from .physics import World 

class Arena(Node):
    """
    the node for setting up the arena
    """

    def __init__(self):
        super().__init__('arena')

        # Setting up the physics module
        self.brick_physics = World([0.0, 0.0, 5.0], 9.8, 0.3, 0.01)
    
        # Create service for placing the brick at a given position
        self.place_srv = self.create_service(Place, 'place', self.place_callback) 

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
        self.brick_tmr = self.create_timer(1/100, self.brick_tmr_callback)

        

    def brick_tmr_callback(self):
        odom_brick_tf = TransformStamped()
        odom_brick_tf.header.frame_id = 'odom'
        odom_brick_tf.child_frame_id = 'brick'
        brick_location = self.brick_physics.brick
        odom_brick_tf.transform.translation.x = brick_location[0]
        odom_brick_tf.transform.translation.y = brick_location[1]
        odom_brick_tf.transform.translation.z = brick_location[2]
    
        self.brick_tf_broadcaster.sendTransform(odom_brick_tf)

        self.brick = Marker()
        self.brick.header.frame_id = "odom"
        self.brick.header.stamp = self.get_clock().now().to_msg()
        self.brick.id = 5
        self.brick.type = Marker.CUBE
        self.brick.action = Marker.ADD
        self.brick.scale.x = 0.15
        self.brick.scale.y = 0.15
        self.brick.scale.z = 0.3
        self.brick.pose.position.x = brick_location[0]
        self.brick.pose.position.y = brick_location[1]
        self.brick.pose.position.z = brick_location[2]
        self.brick.pose.orientation.x = 0.707
        self.brick.pose.orientation.y = 0.0
        self.brick.pose.orientation.z = 0.0
        self.brick.pose.orientation.w = 0.707
        self.brick.color.r = 1.0
        self.brick.color.g = 0.0
        self.brick.color.b = 0.0
        self.brick.color.a = 1.0
        self.brick_publisher.publish(self.brick)
        
        
        
    def place_callback(self, request, response):
        print(request.brick_position) 
        self.brick_physics.brick= request.brick_position
        return response


def start_arena(args=None):
    rclpy.init(args=args)
    node = Arena()
    rclpy.spin(node)
    rclpy.shutdown()
