#!/usr/bin/env python3
from ur_msgs.srv import SetIO
from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node
import sys
from time import sleep

class GripperMove(Node):

    def __init__(self):
        super().__init__('GripperClose')
        self.req = SetIO.Request()
        self.req2 = SetIO.Request()
        self.cli = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.buffer=False
        self.i=0
        self.subscription = self.create_subscription(Bool, 'grip', self.check_grip_state, 10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Subscribing Grip')
        # self.subscription_encoder = self.create_subscription(Bool, 'maxopenclose', self.check_encoder_state, 10)
        # self.subscription_encoder  # prevent unused variable warning
        # self.get_logger().info('Subscribing Encoder')
        #deactivate DOs initial
        self.req.fun = 1 
        self.req.pin = 16
        self.req.state = 0.0
        self.future = self.cli.call_async(self.req)
        self.req2.fun = 1 
        self.req2.pin = 17
        self.req2.state = 1.0
        self.future = self.cli.call_async(self.req2)
        #activate DOs initial
        self.req.fun = 1 
        self.req.pin = 16
        self.req.state = 1.0
        self.future = self.cli.call_async(self.req)
        self.req2.fun = 1 
        self.req2.pin = 17
        self.req2.state = 1.0
        self.future = self.cli.call_async(self.req2)
        sleep(0.1)#necessary to pass the static friction
        self.get_logger().info('Initial Sleeping done')


    def check_grip_state(self, msg):
        if msg.data == True:
            self.open()
        else:
            self.get_logger().info('CurrentStop')
            self.get_logger().info('I heard Gripper: "%s"' % msg.data)
            self.stop_open()
    
    def check_encoder_state(self, msg):
        if msg.data == True:
            self.open()
        else:
            self.get_logger().info('EncoderStop')
            self.get_logger().info('I heard Encoder: "%s"' % msg.data)
            self.stop_open()
        
    def open(self):
        #self.get_logger().info('opening')
        if self.buffer==False:
            self.buffer=True
            self.get_logger().info('set buffer: "%s"' % self.buffer)
            self.req.fun = 1 
            self.req.pin = 17
            self.req.state = 1.0
            self.future = self.cli.call_async(self.req)
            sleep(0.2)
            #rclpy.spin_until_future_complete(self, self.future)
            self.req2.fun = 1 
            self.req2.pin = 16
            self.req2.state = 1.0
            self.future = self.cli.call_async(self.req2)
            #rclpy.spin_until_future_complete(self, self.future)
        else:
            pass

    def stop_open(self):
        self.get_logger().info('open')
        if self.buffer==True:
            self.buffer=False
            self.get_logger().info('set buffer: "%s"' % self.buffer)
            self.req.fun = 1 
            self.req.pin = 16 #first deactivate 16,otherwise the gripper would open a bit
            self.req.state = 0.0
            self.future = self.cli.call_async(self.req)
            #rclpy.spin_until_future_complete(self, self.future)
            sleep(0.1)
            self.req2.fun = 1 
            self.req2.pin = 17
            self.req2.state = 0.0
            self.future = self.cli.call_async(self.req2)
            #rclpy.spin_until_future_complete(self, self.future)
            sys.exit()
        else:
            pass


def main(args=None):
    rclpy.init(args=args)
    GraspClose = GripperMove()
    rclpy.spin(GraspClose)
    GraspClose.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()