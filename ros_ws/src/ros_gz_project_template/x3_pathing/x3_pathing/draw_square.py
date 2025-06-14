import rclpy
from rclpy.node import Node


'''
    We're going to implement an open loop controller/FSM to have the turtlebot
    draw a square on the screen. Below I have commented parts of the code that
    you need to fill in to make the logic complete.
'''

# We have to use the geometry_msgs/msg/Twist to control robots
# Write in here what the correct import should be
from geometry_msgs.msg import Twist
import math

class DrawSquare(Node):

    def __init__(self):
        # Init the node with a name (this is the name that appears when running)
        # 'ros2 node list'
        super().__init__('draw_square')
        
        # Remember that the 'create_publisher' function takes in three arguments
        # Message Type | Topic Name | Queue Length
        # Fill in those values here
        self.publisher_ = self.create_publisher(Twist, '/x3/alt_controller/cmd_vel', 10)

        # Functions running at 1Hz
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Think of this flag as a FSM, 
        # or that the turtle has two modes of operation.
        # The robot is either turning in place, or not turning in place
        # i.e. moving forward.
        self.turning = False

        self.i = 0

        # Let's create two messages to send to the robot depending
        # on the mode its in.
        # What should their type be (should the same as 'Import' above)

        # If I want the robot to move "1m forward" what should
        # the speed be, given the timer is running at 1hz?
        # (Note that values are in m/s)
        # Along which axis should I move in?
        self.forward_msg = Twist()
        self.forward_msg.linear.x = 1.0
        self.forward_msg.linear.y = 0.0
        self.forward_msg.linear.z = 0.0
        self.forward_msg.angular.x = 0.0
        self.forward_msg.angular.y = 0.0
        self.forward_msg.angular.z = 0.0

        # What if I want the robot to turn 90 degrees?
        # Along which axis?
        # (Note that values are in rad/s)
        self.turn_msg = Twist()
        self.turn_msg.linear.x = 0.0
        self.turn_msg.linear.y = 0.0
        self.turn_msg.linear.z = 0.0
        self.turn_msg.angular.x = 0.0
        self.turn_msg.angular.y = 0.0
        self.turn_msg.angular.z = math.pi / 6  # Set angular velocity in rad/s

    # Callback for the events
    def timer_callback(self):
        # If robot is turining
        if (self.turning):
            # Call publisher here
            self.publisher_.publish(self.turn_msg)
            self.get_logger().info('Robot is Turning!')
            self.i += 1
            if (self.i >= 3):
                self.i = 0
                self.turning = not self.turning

        
        else:
            # Call publisher here
            self.publisher_.publish(self.forward_msg)   
            # Get logger function call similar to 'cout' in C++
            self.get_logger().info('Robot is Moving Forward 1!')
            # or 'print()' in python
            # Flip the mode of the robot
            self.turning = not self.turning


def main(args=None):
    rclpy.init(args=args)

    draw_square = DrawSquare()

    # Spin function *important*
    # makes sure that the program does not terminate
    # immediately
    rclpy.spin(draw_square)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    draw_square.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()