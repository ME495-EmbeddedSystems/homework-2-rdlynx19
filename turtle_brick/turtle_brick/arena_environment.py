import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class Arena(Node):
    """
    the node for setting up the arena
    """

    def __init__(self):
        super().__init__('arena')

        # create a marker publisher
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', markerQoS)

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





def start_arena(args=None):
    rclpy.init(args=args)
    node = Arena()
    rclpy.spin(node)
    rclpy.shutdown()
