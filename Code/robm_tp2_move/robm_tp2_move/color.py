from time import clock_getres
import rclpy
from rclpy.node import Node

from robm_interfaces.msg import RoverMotorsCmd

from robm_interfaces.msg import Color

import math

class ColorNode(Node):
    """ROS2 node that makes the robot run forward during 2 seconds."""
    def __init__(self):
        # Initialize the node with the name 'couleur'
        super().__init__('avance')
        # Count the number of published messages
        self.i = 0
        # The number of messages to publish
        self.n = 2000

          # Create a subscriber to the 'color' topic
        self.sub_color = self.create_subscription(Color, 'color', self.color_callback, 10)

        self.ref_color_map = {
            "red" : [40.0,9.0,8.0],
            "green" : [6.0,33.0,15.0],
            "blue" : [1.0,13.0,64.0],
            "black" : [7.0,16.0,27.0],
            "white" : [4.0,17.0,31.0],
            "grey" : [7.0,17.0,26.0]
        }

        # Create a publisher to the 'cmd_motors' topic
        self.pub_motors = self.create_publisher(RoverMotorsCmd, 'cmd_motors', 10)
        # Create a timer to publish a message every 0.1 seconds
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    

 

    def calculate_color_distance(self, color1, color2):
        """Calculates the Euclidean distance between two RGB colors."""
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(color1, color2)))

    def find_closest_color(self, target_color, color_map):
        """Finds the closest color in the given color map to the target color."""
        closest_color = None
        min_distance = float('inf')

        for color_name, color_values in color_map.items():
            distance = self.calculate_color_distance(target_color, color_values)
            if distance < min_distance:
                min_distance = distance
                closest_color = color_name

        return closest_color


    def color_callback(self, msg: Color):
        # TODO: Get RGB data from color sensor
        #print("color")
        
        self.target_rgb_color = [msg.r,msg.g,msg.b]
        self.closest_color = self.find_closest_color(self.target_rgb_color, self.ref_color_map)
        print(f"{self.closest_color}")

    def timer_callback(self):
        """Callback function 10 Hz timer"""
        # Instanciate a RoverMotorsCmd message
        msg = RoverMotorsCmd()
       
       
        #self.closest_color = self.find_closest_color(self.target_rgb_color, self.ref_color_map)
       
       

        match self.closest_color:
            case "blue":
                msg.front_left=-0.3
                msg.front_right=+0.3
                msg.rear_left=-0.3
                msg.rear_right=0.3

            case "red":
                msg.front_left=1.0
                msg.front_right=1.0
                msg.rear_left=1.0
                msg.rear_right=1.0

            case "green":
                msg.front_left=0.2
                msg.front_right=0.2
                msg.rear_left=0.2
                msg.rear_right=0.2

            case "black":
                msg.front_left=0.0
                msg.front_right=0.0
                msg.rear_left=0.0
                msg.rear_right=0.0

            case "grey":
                msg.front_left=0.5
                msg.front_right=0.5
                msg.rear_left=0.5
                msg.rear_right=0.5
            case "white":
                msg.front_left=0.5
                msg.front_right=0.5
                msg.rear_left=0.5
                msg.rear_right=0.5
            case _:
                msg.front_left=0.5
                msg.front_right=0.5
                msg.rear_left=0.5
                msg.rear_right=0.5




        # Publish the message
        self.pub_motors.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg)
        # Count the number of published messages
        self.i += 1
        # Stop the timer after 20 messages and exit
        if self.i == self.n:
            self.timer.cancel()
            self.get_logger().info('Stop.')
            exit(0)


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    # Create the node
    color = ColorNode()

    # Run the node until it's stopped
    rclpy.spin(color)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    color.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
