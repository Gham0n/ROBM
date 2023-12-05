import rclpy
from rclpy.node import Node

from robm_interfaces.msg import RoverMotorsCmd
from sensor_msgs.msg import Range

class Telemetre(Node):
    """ROS2 node that makes the robot run forward during 2 seconds."""
    def __init__(self):
        # Initialize the node with the name 'couleur'
        super().__init__('avance')
        # Count the number of published messages
        self.i = 0
        # The number of messages to publish
        self.n = 200

        # Create a subscriber to the 'tof' topic
        self.sub_tof = self.create_subscription(Range, 'tof', self.tof_callback, 10)
        self.current_range = 0.0
        
        # Create a publisher to the 'cmd_motors' topic
        self.pub_motors = self.create_publisher(RoverMotorsCmd, 'cmd_motors', 10)
        # Create a timer to publish a message every 0.1 seconds
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def tof_callback(self, msg: Range):
        print("tof")
        tmp = round(msg.range*100)
        print("range:",tmp)
        self.current_range = tmp

    def timer_callback(self):
        """Callback function 10 Hz timer"""
        # Instanciate a RoverMotorsCmd message
        msg = RoverMotorsCmd()
        
        #avance
        if(self.current_range > 50):
            msg.front_left=0.5
            msg.front_right=0.5
            msg.rear_left=0.5
            msg.rear_right=0.5
        #recule
        elif(self.current_range <= 20):
                msg.front_left=-0.5
                msg.front_right=-0.5
                msg.rear_left=-0.5
                msg.rear_right=-0.5
        #tourne
        elif (20 < self.current_range <= 50) :
            msg.front_left=0.5
            msg.front_right=-0.5
            msg.rear_left=-0.5
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
    telemetre = Telemetre()

    # Run the node until it's stopped
    rclpy.spin(telemetre)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    telemetre.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
