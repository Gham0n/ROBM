import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from robm_interfaces.msg import RoverMotorsCmd

class VelToMotor(Node):
    """ROS2 node that converts a Twist (speed) to motor commands of 4 mecanum wheels."""
    def __init__(self):
        # Initialize the node with the name 'couleur'
        super().__init__('vel_to_motor')
        # Create a publisher to the 'cmd_motors' topic
        self.pub_motors = self.create_publisher(RoverMotorsCmd, 'cmd_motors', 10)
        # Create a subscriber to the 'vel' topic
        self.sub_vel = self.create_subscription(Twist, 'vel', self.vel_callback, 10)
    
    def vel_callback(self, msg: Twist):
        """Callback function for the 'vel' topic subscriber"""
        # Extract the linear and angular speeds from the message
        vx = msg.linear.x
        vy = msg.linear.y
        w = msg.angular.z
        w = w * 0.5

        # Instanciate a RoverMotorsCmd message
        cmd_msg = RoverMotorsCmd()
        
        print("vx:",vx)
        print("vy:",vy)
        print("w:",w)

        cmd_msg.front_left=vx-vy-w
        cmd_msg.front_right=vx+vy+w
        cmd_msg.rear_left=vx+vy-w
        cmd_msg.rear_right=vx-vy+w


        # Publish the message
        self.pub_motors.publish(cmd_msg)
        self.get_logger().info('Publishing: "%s"' % cmd_msg)


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    # Create the node
    vel_to_motor = VelToMotor()

    # Run the node until it's stopped
    rclpy.spin(vel_to_motor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vel_to_motor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
