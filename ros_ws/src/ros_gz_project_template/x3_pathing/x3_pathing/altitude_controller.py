import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

from nav_msgs.msg import Odometry


class AltitudeController(Node):
    def __init__(self):
        super().__init__('altituide_controller')

        #PID Constants
        self.kp = 1.0
        self.ki = 0.0007
        self.kd = 0.3
        self.twist = Twist()


        self.target_alt = 1.5 # 1.5 meters hardcoded
        # TODO add ability to take new heights

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = self.get_clock().now()

        #Publish to cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/x3/cmd_vel', 10)
        
        # TODO utilize ICP + Localization odom
        self.odom_sub = self.create_subscription(Odometry, 'x3/altimeter/pose', self.odom_callback, 10)
        self.cmd_sub = self.create_subscription(TwistStamped, 'cmd_vel', self.cmd_callback, 10)

    def odom_callback(self, msg):
        # Get the current altitude
        cur_alt = msg.pose.pose.position.z
        
        # Calculate error from target
        error = self.target_alt - cur_alt

        # Get current time and calculate delta t
        cur_time = self.get_clock().now()
        dt = (cur_time - self.prev_time).nanoseconds / 1e9        
        
        # Set prev time to current time
        self.prev_time = cur_time
        
        # Increment integral value
        self.integral += error * dt

        # Calculate derivative value
        if (dt > 0):
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0
        self.prev_error = error

        # PID Formula
        velocity_z = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Limit velocity
        velocity_z = max(min(velocity_z, 1.0), -1.0)

        # Publish Twist
        injectedTwist = self.twist
        self.twist.linear.z = velocity_z
        self.cmd_pub.publish(injectedTwist)

    def cmd_callback(self, msg):
        #Get the current velocity cmd
        self.twist = msg.twist
def main(args=None):
    rclpy.init(args=args)
    node = AltitudeController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        

