import rclpy
from rclpy.node import Node
import tf2_ros
from turtlesim.msg import Pose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class Catcher(Node):
    """
    Node to catch the brick
    """
    def __init__(self):
        super().__init__('catcher')

        self.max_vel = self.declare_parameter("max_velocity", 5.0)
        self.platform_height = self.declare_parameter("platform_height", 0.7)
        self.wheel_radius = self.declare_parameter("wheel_radius", 0.15)

        self.tmr_to_brick = self.create_timer(1/250, self.tmr_to_brick_callback)

        self.tf_buffer = Buffer()
        

        self.turtlesim_pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.turtlesim_pose_callback, 1
        )

        self.turtlesim_current_pose = Pose()
        self.turtlesim_current_pose.x = 5.54
        self.turtlesim_current_pose.y = 5.54


    def tmr_to_brick_callback(self):
        """
        Compute the time it will take to reach the brick
        """
        plat_height = self.get_parameter('platform_height').value
        wheel_radius = self.get_parameter('wheel_radius').value
        max_velocity = self.get_parameter('max_velocity').value
        try:
            odom_brick_lookup = self.tf_buffer.lookup_transform('odom', 'brick', rclpy.time.Time())
            current_brick_height = odom_brick_lookup.transform.translation.z

            dist_to_platform = current_brick_height - (plat_height + (wheel_radius*2) + 0.2)
            
            planar_dist_to_brick = ((self.turtlesim_current_pose.x - odom_brick_lookup.transform.translation.x)**2 + (self.turtlesim_current_pose.y - odom_brick_lookup.transform.translation.y)**2)**0.5
            
            minimum_time_to_brick = planar_dist_to_brick/max_velocity           

            if((dist_to_platform/max_velocity) > minimum_time_to_brick):
                self.get_logger().info("Cant reach the fucking brick")
            else:
                self.get_logger().info(f"I can reach the brick at position x: {odom_brick_lookup.transform.translation.x} y: {odom_brick_lookup.transform.translation.y}")
    


        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().info(f'Lookup exception: {e}')
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f'Connecticvity exception: {e}')
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().info(f'Extrapolation exception: {e}') 

    def turtlesim_pose_callback(self, turtlesim_pose_msg):
        self.turtlesim_current_pose = turtlesim_pose_msg


def catch_brick(args=None):
    rclpy.init(args=args)
    node = Catcher()
    rclpy.spin(node)
    rclpy.shutdown()

