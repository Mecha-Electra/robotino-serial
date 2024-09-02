#/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        rate = self.create_rate(0.1)
        msg = Twist()
        while self.ok():
            msg.linear.x =2.0
            msg.angular.z=1.0
            self.publisher.publish(msg)
            rate.sleep()
        

def main(args=None):
    rclpy.init(args=args)

    turtle_controller = TurtleController()

    rclpy.spin(turtle_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()