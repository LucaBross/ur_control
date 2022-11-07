from ur_msgs.srv import SetIO
from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node
import sys
from time import sleep

class GripperMove(Node):

    def __init__(self):
        super().__init__('GripperClose')
        self.cli = self.create_client(SetIO, '/io_and_status_controller/set_io')
        self.subscription = self.create_subscription(Bool, 'grip', self.check_grip_state, 10)
        self.subscription  # prevent unused variable warning
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetIO.Request()
        self.req2 = SetIO.Request()
        self.buffer=False
        self.i=0
        #deactivate DOs initial
        self.req.fun = 1 
        self.req.pin = 16
        self.req.state = 0.0
        self.future = self.cli.call_async(self.req)
        self.req2.fun = 1 
        self.req2.pin = 17
        self.req2.state = 0.0
        self.future = self.cli.call_async(self.req2)

    def check_grip_state(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.get_logger().info('i=: "%s"' % self.i)
        if msg.data == True:
            self.i=0
            self.close()
        else:
            sleep(0.02)
            self.i=self.i+1
            if self.i >= 3:
                self.stop_close()
        
    def close(self):
        self.get_logger().info('closing')
        if self.buffer==False :
            self.buffer=True
            self.get_logger().info('set buffer: "%s"' % self.buffer)
            self.req.fun = 1 
            self.req.pin = 17
            self.req.state = 1.0
            self.future = self.cli.call_async(self.req)
            #rclpy.spin_until_future_complete(self, self.future)
            self.req2.fun = 1 
            self.req2.pin = 16
            self.req2.state = 1.0
            self.future = self.cli.call_async(self.req2)
            #rclpy.spin_until_future_complete(self, self.future)
        else:
            pass

    def stop_close(self):
        self.get_logger().info('object_detected')
        if self.buffer==True:
            self.buffer=False
            self.get_logger().info('set buffer: "%s"' % self.buffer)
            self.req.fun = 1 
            self.req.pin = 16 #first deactivae 16,otherwise the gripper would open a bit
            self.req.state = 0.0
            self.future = self.cli.call_async(self.req)
            #rclpy.spin_until_future_complete(self, self.future)
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


