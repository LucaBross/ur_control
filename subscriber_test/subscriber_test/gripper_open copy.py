from ur_msgs.srv import SetIO
from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node
import sys
from time import sleep

class GripperMove(Node):

    def __init__(self):
        super().__init__('GripperOpen')
        self.cli = self.create_client(SetIO, '/io_and_status_controller/set_io')
        self.subscription = self.create_subscription(Bool, 'grip', self.check_grip_state, 10)
        self.subscription  # prevent unused variable warning
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetIO.Request()
        self.buffer=False
        self.i=0
        #deactivate DO initial
        self.req.fun = 1 
        self.req.pin = 16
        self.req.state = 0.0
        self.future = self.cli.call_async(self.req)

    def check_grip_state(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.get_logger().info('i=: "%s"' % self.i)
        if msg.data == True:
            self.i=0
            self.open()
        else:
            sleep(0.02)
            self.i=self.i+1
            if self.i >= 5:
                self.stop_open()
        
    def open(self):
        self.get_logger().info('opening')
        if self.buffer==False :
            self.buffer=True
            self.get_logger().info('set buffer: "%s"' % self.buffer)
            self.req.fun = 1 
            self.req.pin = 16
            self.req.state = 1.0
            self.future = self.cli.call_async(self.req)
            #rclpy.spin_until_future_complete(self, self.future)
        else:
            pass

    def stop_open(self):
        self.get_logger().info('stop_opening')
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



