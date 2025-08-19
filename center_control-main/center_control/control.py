from ast import In
from platform import node
from tkinter import SEL_LAST
from turtle import pos
import rclpy    
from rclpy.node import Node   
from std_msgs.msg import Int16   
import numpy as np     

class control_c(Node):
    def __init__(self):
        super().__init__('pos_camera_to_arm')
        self.action_s = self.create_subscription(Int16, '/action', self.action_call, 10)
        self.start_s = self.create_subscription(Int16, '/start', self.start_call, 10)
        self.car_fin = self.create_subscription(Int16, '/fin', self.fin_call, 10)

        # self.stop_s = self.create_subscription(Int16, '/stop', self.stop, 10)

        self.service_p = self.create_publisher(Int16, '/service', 10)
        self.timer = self.create_timer(0.1, self.timer_call)
        self.timer.cancel()
        self.action = None
        self.service = None
        self.start = None
        self.i = 2
    
    def fin_call(self):
        if self.action == -1:
            self.action = 1

    def action_call(self, msg):
        self.action = msg.data

    # def stop(self, msg):
    #     if msg.data == 1:
    #         self.service_p.publish(-5)
    #         self.action = None
    #         self.service = None
    #         self.start = None

    def start_call(self, msg):
        if self.start == -1 and msg.data == 1:
            self.timer.reset()
            
        elif msg.data == -1 and self.start == 1:
            self.timer.cancel()
            # if self.action == 3
            self.action = None
            self.service = None
            self.start = None
            
        self.start = msg.data

    def timer_call(self):
        msg = Int16()

        if self.action is None:
        #     msg.data = -1
        # elif self.action == 1:
            msg.data = 2 
        elif self.action == 2:
            self.i += 1
            msg.data = 3           
        elif self.action == 3:
            msg.data = -4
            print("finish")
        self.service_p.publish(msg)

def main():
    rclpy.init()
    node = control_c()
    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown(node)


if __name__ == '__main__':
    main()