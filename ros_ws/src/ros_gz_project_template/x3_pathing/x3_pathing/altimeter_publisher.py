import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ros_gz_interfaces.msg import Altimeter

#This class subscribes to altimeter data and changes the format to a pose to improve localization accuracy

class AltimeterPublisher(Node):
    def __init__(self):
        super().__init__('altimeter_publisher')
        
        # Publisher
        self.publisher_ = self.create_publisher(Pose, 'x3/altimeter/pose', 10)
        
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
        poseHeight = Pose()
        poseHeight.position.z = altitudeData.vertical_position
        self.publisher_.publish(poseHeight)

def main(args=None):
    rclpy.init(args=args)
    node = AltimeterPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
