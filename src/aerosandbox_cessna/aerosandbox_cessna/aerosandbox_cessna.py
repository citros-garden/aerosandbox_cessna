import rclpy
from rclpy.node import Node

from . import util_aerosandbox_cessna

from std_msgs.msg import Float64MultiArray

class aerosandbox_cessna(Node):

    def __init__(self):
        super().__init__('dynamics')
        
        self.state_pub = self.create_publisher(Float64MultiArray , '/aerosandbox_cessna/state', 10)

        self.get_logger().info(f"Starting aerosandbox_cessna version = 0.0.0")

        #Defining inputs
        self.declare_parameter('h_0', 1000.0) # altitude, m
        self.declare_parameter('v_0', 107.0) # velocity, knots

        self.declare_parameter('publish_freq', 10.0)   
        
        self.h_0 = self.get_parameter('h_0').get_parameter_value().double_value
        self.v_0 = self.get_parameter('v_0').get_parameter_value().double_value

        # Defining encounter for publisher
        self.i = 0        

        #################################
        # Calling simulation function using parameters declared above
        self.res_x, self.res_h, self.res_v =  util_aerosandbox_cessna.run(self,
                                      h_0 = self.h_0,
                                      v_0 = self.v_0)
        
        self.state_msg = Float64MultiArray()
        timer_period = 1/self.get_parameter('publish_freq').get_parameter_value().double_value  # frequency of publishing
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):

        self.state_msg.data = [self.res_x[self.i], -self.res_h[self.i], self.res_v[self.i]]

        self.state_pub.publish(self.state_msg)
        self.get_logger().info(f"Publishing = {self.state_msg.data}")

        self.i += 1
        if self.i==len(self.res_x):
            self.get_logger().info('All data published successfully')
            exit()
        
        
def main(args=None):
    rclpy.init(args=args)
    dynamics = aerosandbox_cessna()
    rclpy.spin(dynamics)
    dynamics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()