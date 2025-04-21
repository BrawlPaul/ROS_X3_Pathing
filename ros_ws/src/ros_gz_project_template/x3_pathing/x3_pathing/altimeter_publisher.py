import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from ros_gz_interfaces.msg import Altimeter
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry


#This class subscribes to altimeter data and changes the format to a pose to improve localization accuracy

class AltimeterPublisher(Node):
    def __init__(self):
        super().__init__('altimeter_publisher')
        
        # Publisher
        self.publisher_ = self.create_publisher(Odometry, 'x3/altimeter/pose', 10)
        
        # Subscriber
        self.subscription = self.create_subscription(
            Altimeter,
            'x3/altimeter',
            self.listener_callback,
            10
        ) 

        self.subscription  # prevent unused variable warning
     
    
    def listener_callback(self, msg):
        altitudeData = msg
        poseHeight = Odometry()
        poseHeight.pose.covariance = [
        0,      0,      0,      0,      0,      0,
        0,      0,      0,      0,      0,      0,
        0,      0,      0,      0,      0,      0,
        0,      0,      0,      0,      0,      0,
        0,      0,      0,      0,      0,      0,
        0,      0,      0,      0,      0,      0
        ]
        poseHeight.pose.pose.position.z = altitudeData.vertical_position
        poseHeight.header.stamp = self.get_clock().now().to_msg()
        poseHeight.header.frame_id = "x3/odom"
        poseHeight.child_frame_id = "x3/base_link"
        self.publisher_.publish(poseHeight)

def main(args=None):
    rclpy.init(args=args)
    node = AltimeterPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
