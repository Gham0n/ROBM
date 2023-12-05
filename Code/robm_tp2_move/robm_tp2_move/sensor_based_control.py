import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from robm_interfaces.msg import Color, RoverMotorsCmd
from sensor_msgs.msg import Range

class SensorBasedControl(Node):
    """ROS2 node that controls the robot based on sensors measurements."""
    def __init__(self):
        # Initialize the node with the name 'couleur'
        super().__init__('sensor_based_control')
        # Create a subscriber to the 'tof' topic
        self.sub_tof = self.create_subscription(Range, 'tof', self.tof_callback, 10)
        # Create a subscriber to the 'color' topic
        self.sub_color = self.create_subscription(Color, 'color', self.color_callback, 10)
        # Create a publisher to the 'vel' topic
        self.pub_motors = self.create_publisher(Twist, 'vel', 10)
        # Create a timer to publish a message every 0.1 seconds
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Range measured by tof sensor
        self.current_range = 0.0
        self.pub_motors = self.create_publisher(RoverMotorsCmd, 'cmd_motors', 10)
        
        
        self.closest_color= "black"
        self.sub_color = self.create_subscription(Color, 'color', self.color_callback, 10)

        self.ref_color_map = {
            "red" : [40.0,9.0,8.0],
            "green" : [6.0,33.0,15.0],
            "blue" : [1.0,13.0,64.0],
            "black" : [7.0,16.0,27.0],
            "white" : [4.0,17.0,31.0],
            "grey" : [7.0,17.0,26.0]
        }

    def tof_callback(self, msg: Range):
        print("tof")
        tmp = round(msg.range*100)
        print("range:",tmp)
        self.current_range = tmp
    
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
        #print("color")
        
        self.target_rgb_color = [msg.r,msg.g,msg.b]
        self.closest_color = self.find_closest_color(self.target_rgb_color, self.ref_color_map)
        print(f"{self.closest_color}")
        
    def timer_callback(self):
        """Callback function 10 Hz timer"""
        # Instanciate a RoverMotorsCmd message
        msg = RoverMotorsCmd()


        match self.closest_color:
            case "blue":
                coefcolor = 0.5

            case "red":
                coefcolor = 1.0

            case "green":
                coefcolor = 0.2

            case "black":
                coefcolor = 0.0

            case "grey":
                coefcolor = 0.5
            case "white":
                coefcolor = 0.5
            case _:
                coefcolor = 0.5

        # TODO: if range measured by tof sensor is lower than 0.1, then move backward
         #avance
        if(self.current_range > 50):
            msg.front_left=coefcolor
            msg.front_right=coefcolor
            msg.rear_left=coefcolor
            msg.rear_right=coefcolor
        #recule
        elif(self.current_range <= 20):
                msg.front_left=-coefcolor
                msg.front_right=-coefcolor
                msg.rear_left=-coefcolor
                msg.rear_right=-coefcolor
        #tourne
        elif (20 < self.current_range <= 50) :
            msg.front_left=coefcolor
            msg.front_right=-coefcolor
            msg.rear_left=-coefcolor
            msg.rear_right=coefcolor

       
        # Publish the message
        self.pub_motors.publish(msg)
       
def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    # Create the node
    sensor_based_control = SensorBasedControl()

    # Run the node until it's stopped
    rclpy.spin(sensor_based_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor_based_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
