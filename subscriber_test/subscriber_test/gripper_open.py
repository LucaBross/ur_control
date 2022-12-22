#!/usr/bin/env python3
from ur_msgs.srv import SetIO
from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node
import sys
from time import sleep

class GripperMove(Node):

    def __init__(self):
        super().__init__('GripperOpen')
        self.req = SetIO.Request()
        self.cli = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.buffer=False
        # self.subscription = self.create_subscription(Bool, 'grip', self.check_grip_state, 10)
        # self.subscription  # prevent unused variable warning
        # self.get_logger().info('Subscribing Grip')
        # self.subscription_encoder = self.create_subscription(Bool, 'maxopenclose', self.check_encoder_state, 10)
        # self.subscription_encoder  # prevent unused variable warning
        # self.get_logger().info('Subscribing Encoder')
        #deactivate DO initial
        self.req.fun = 1 
        self.req.pin = 16
        self.req.state = 0.0
        self.future = self.cli.call_async(self.req)
        #activate DO initial
        self.req.fun = 1 
        self.req.pin = 16
        self.req.state = 1.0
        self.future = self.cli.call_async(self.req)
        sleep(0.5)#necessary to pass the static friction
        self.get_logger().info('Initial Sleeping done')
        # #deactivate DO 
        self.req.fun = 1 
        self.req.pin = 16
        self.req.state = 0.0
        self.future = self.cli.call_async(self.req)
        sys.exit()

    def check_encoder_state(self, msg):
        if msg.data == True:
            self.close()
        else:
            self.get_logger().info('EncoderStop')
            self.get_logger().info('I heard Encoder: "%s"' % msg.data)
            self.stop_close()

    def check_grip_state(self, msg):
        if msg.data == True:
            self.close()
        else:
            self.get_logger().info('CurrentStop')
            self.get_logger().info('I heard Gripper: "%s"' % msg.data)
            self.stop_close()
        
    def close(self):
        #self.get_logger().info('closing')
        if self.buffer==False:
            self.buffer=True
            self.get_logger().info('set buffer: "%s"' % self.buffer)
            self.req.fun = 1 
            self.req.pin = 16
            self.req.state = 1.0
            self.future = self.cli.call_async(self.req)
            #rclpy.spin_until_future_complete(self, self.future)
        else:
            pass

    def stop_close(self):
        self.get_logger().info('stop_closing')
        if self.buffer==True:
            self.buffer=False
            self.get_logger().info('set buffer: "%s"' % self.buffer)
            self.req.fun = 1 
            self.req.pin = 16
            self.req.state = 0.0
            self.future = self.cli.call_async(self.req)
            #rclpy.spin_until_future_complete(self, self.future)
            sys.exit()
        else:
            pass


def main(args=None):
    rclpy.init(args=args)
    GraspOpen = GripperMove()
    rclpy.spin(GraspOpen)
    GraspOpen.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

