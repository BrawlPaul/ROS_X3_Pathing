import asyncio
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionServer
from x3_interfaces.action import SetHeight

from nav_msgs.msg import Odometry


class AltitudeController(Node):
    def __init__(self):
        super().__init__('altituide_controller')

        #PID Constants
        self.kp = 1.0
        self.ki = 0.0007
        self.kd = 0.3
        self.twist = Twist()


        self.target_alt = 1.5
        self.cur_alt = 0

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = self.get_clock().now()

        #Publish to cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/x3/cmd_vel', 10)

        #Use altimeter value as height        
        self.odom_sub = self.create_subscription(Odometry, 'x3/altimeter/pose', self.odom_callback, 10)
        self.cmd_sub = self.create_subscription(TwistStamped, 'cmd_vel', self.cmd_callback, 10)

        #Create action server to recv height requests
        self._action_server = ActionServer(
            self,
            SetHeight,
            '/set_height',
            self.set_height
        )

    def set_height(self, goal_handle):
            self.target_alt = goal_handle.request.target_height
            self.get_logger().info(f"Received goal request to set height to {self.target_alt} meters.")

            result_msg = SetHeight.Result()
            result_msg.final_height = self.target_alt
            goal_handle.succeed()

            self.get_logger().info(f"Successfully set height to {self.target_alt} meters.")

            return result_msg


    def odom_callback(self, msg):
        #self.get_logger().info("ODOM CALLBACK")
        # Get the current altitude
        self.cur_alt = msg.pose.pose.position.z
        
        # Calculate error from target
        error = self.target_alt - self.cur_alt

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

        

