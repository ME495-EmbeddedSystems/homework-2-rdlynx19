import rclpy
import rclpy.duration
from rclpy.node import Node
import tf2_ros
from turtlesim.msg import Pose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration


class Catcher(Node):
    """
    Node to catch the brick
    """
    def __init__(self):
        super().__init__('catcher')

        self.max_vel = self.declare_parameter("max_velocity", 3.0)
        self.platform_height = self.declare_parameter("platform_height", 0.7)
        self.wheel_radius = self.declare_parameter("wheel_radius", 0.15)

        self.tmr_to_brick = self.create_timer(1/250, self.tmr_to_brick_callback)

        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer, self)
        

        self.turtlesim_pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.turtlesim_pose_callback, 1
        )

        self.turtlesim_current_pose = Pose()
        self.turtlesim_current_pose.x = 5.54
        self.turtlesim_current_pose.y = 5.54

        self.goal_pose_publisher = self.create_publisher(PoseStamped, "goal_pose",10)


        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_publisher = self.create_publisher(Marker, 'text_marker', markerQoS)

        self.brick_z_positions = []
            
        self.timer_iteration = 0

    def tmr_to_brick_callback(self):
        """
        Compute the time it will take to reach the brick
        """
        plat_height = self.get_parameter('platform_height').value
        wheel_radius = self.get_parameter('wheel_radius').value
        max_velocity = self.get_parameter('max_velocity').value
        try:
            odom_brick_lookup = self.tf_buffer.lookup_transform('world', 'brick', rclpy.time.Time())
            current_brick_height = odom_brick_lookup.transform.translation.z

            self.brick_z_positions.append(current_brick_height)

            if(self.timer_iteration == 10):
                if(self.brick_z_positions[-2] - self.brick_z_positions[-1] >= 0.0000784):
                    dist_to_platform = current_brick_height - (plat_height + (wheel_radius*2) + 0.2)
                    planar_dist_to_brick = (self.turtlesim_current_pose.x - (odom_brick_lookup.transform.translation.x ))**2 + (self.turtlesim_current_pose.y - (odom_brick_lookup.transform.translation.y)**2)**0.5
                    
                    minimum_time_to_brick = planar_dist_to_brick/max_velocity     

                    if(dist_to_platform > 0.0):
                        time_to_platform = ((dist_to_platform * 2)/9.8)**0.5
                        if(time_to_platform > minimum_time_to_brick):
                            
                            # if(self.timer_iteration == 1):
                                self.display_msg_marker()
                                self.get_logger().info("Cant reach the  brick")
                                
                        else:
                            # if(self.timer_iteration == 1):
                            self.get_logger().info(f"I can reach the brick at position x: {odom_brick_lookup.transform.translation.x} y: {odom_brick_lookup.transform.translation.y}")
                            goal_pose = PoseStamped()
                            goal_pose.pose.position.x = odom_brick_lookup.transform.translation.x
                            goal_pose.pose.position.y = odom_brick_lookup.transform.translation.y 
                            self.goal_pose_publisher.publish(goal_pose)
            try:
                odom_platform_lookup = self.tf_buffer.lookup_transform('odom', 'platform', rclpy.time.Time())
            
                if(odom_brick_lookup.transform.translation.z - odom_platform_lookup.transform.translation.z <= 0.150):
                    center_pose = PoseStamped()
                    center_pose.pose.position.x = 5.54
                    center_pose.pose.position.y = 5.54
                    self.goal_pose_publisher.publish(center_pose)
            except tf2_ros.LookupException as e:
            # the frames don't exist yet
                self.get_logger().info(f'Lookup exception: {e}')
            except tf2_ros.ConnectivityException as e:
                # the tf tree has a disconnection
                self.get_logger().info(f'Connectivity exception: {e}')
            except tf2_ros.ExtrapolationException as e:
                # the times are two far apart to extrapolate
                self.get_logger().info(f'Extrapolation exception: {e}') 



        
        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().info(f'Lookup exception: {e}')
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f'Connecticvity exception: {e}')
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().info(f'Extrapolation exception: {e}') 

    
        

        
        self.timer_iteration += 1
        if(self.timer_iteration > 100):
            self.timer_iteration = 11


    def display_msg_marker(self):
        self.m = Marker()
        self.m.header.frame_id = "world"
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
        self.m.text = "Unreachable"
        self.marker_publisher.publish(self.m)

    def turtlesim_pose_callback(self, turtlesim_pose_msg):
        self.turtlesim_current_pose = turtlesim_pose_msg


def catch_brick(args=None):
    rclpy.init(args=args)
    node = Catcher()
    rclpy.spin(node)
    rclpy.shutdown()

