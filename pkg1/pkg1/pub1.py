import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Publisher(Node):
    def __init__(self):
        super().__init__("Publisher")
        self.publisher=self.create_publisher(String,'topic',10)
        time_period=0.5
        self.timer=self.create_timer(time_period,self.timer_callback)
        self.i=0

    def timer_callback(self):
        msg=String()
        msg.data='Hello Again ROS %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info("Publishinig:%s"%msg.data)
        self.i+=1


def main(args=None):
    rclpy.init(args=args)
    node=Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
