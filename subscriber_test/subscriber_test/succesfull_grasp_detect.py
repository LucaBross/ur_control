import rclpy
from rclpy.node import Node

from ur_msgs.msg import ToolDataMsg
from std_msgs.msg import Bool
import csv
import time

class MinimalPubSub(Node):

    def __init__(self):
        super().__init__('succesful_grasp_detect')
        global Data
        Data=[]
        self.publisher_ = self.create_publisher(Bool, 'grip', 5)
        self.subscription = self.create_subscription(
            ToolDataMsg,
            '/io_and_status_controller/tool_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.declare_parameter('thresh', 0.6)
        thresh=self.get_parameter('thresh').get_parameter_value().double_value
        self.get_logger().info('Current Threshhold: "%s"' % thresh)

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.tool_current)
        # self.get_logger().info('Current Threshhold: "%s"' % self.current_threshhold)
        thresh=self.get_parameter('thresh').get_parameter_value().double_value
        Data.append(str(msg.tool_current))
        if msg.tool_current>=thresh:
            self.current_over_threshold()
            self.get_logger().info('True_Publishing: "%s"' % msg.tool_current)
            self.get_logger().info('Current Threshhold: "%s"' % thresh)
        else:
            self.current_below_threshhold()

    def current_over_threshold(self):
        msg = Bool()
        msg.data = False
        self.publisher_.publish(msg)
        #self.get_logger().info('False_Publishing: "%s"' % msg.data)
    
    def current_below_threshhold(self):
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)
        #self.get_logger().info('True_Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    minimal_pubsub= MinimalPubSub()
    rclpy.spin(minimal_pubsub)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_pubsub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

