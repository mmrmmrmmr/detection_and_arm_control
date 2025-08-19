from ast import In
from platform import node
from re import S
from tkinter import SEL_LAST
from turtle import pos
import rclpy    
from rclpy.node import Node   
from std_msgs.msg import Int16, Float64MultiArray, Int16MultiArray
import numpy as np     
# from geometry_msgs.msg import Twist 

class arm_auto(Node):
    def __init__(self):
        super().__init__('pos_camera_to_arm')
        self.pos_s = self.create_subscription(Float64MultiArray, '/pos_in_arm', self.pos_call, 10)
        self.service_s = self.create_subscription(Int16, '/service', self.service_call, 10)
        self.publisher_service1 = self.create_publisher(Float64MultiArray, '/arm_pos', 10)
        self.publisher_service2 = self.create_publisher(Int16MultiArray, '/arm_close_gamma', 10)

        # self.action_s = self.create_subscription(Int16, '/action', self.start_call, 10)
        
        # self.car_fin = self.create_subscription(Int16, '/fin', self.fin_call, 10)

        # self.stop_s = self.create_subscription(Int16, '/stop', self.stop, 10)

        self.action_p = self.create_publisher(Int16, '/action', 10)
        # self.timer = self.create_timer(0.1, self.timer_call)
        # self.timer.cancel()
        self.pos = None
        self.timer = 1
        # self.start = None
    
    # def fin_call(self):
    #     if self.action == -1:
    #         self.action = 1

    def pos_call(self, msg):
        self.pos = msg.data
        

    # def stop(self, msg):
    #     if msg.data == 1:
    #         self.service_p.publish(-5)
    #         self.action = None
    #         self.service = None
    #         self.start = None

    def service_call(self, msg):
        if  msg.data == 3:
            if self.pos is None:
                self.move_arm([0, 15, 20], 0)
                self.timer = 1
            else:
                self.timer += 1
                if self.timer < 21:
                    self.move_arm(self.pos, -30)
                elif self.timer < 31:
                    self.move_arm(self.pos, -30, 1000, 215, 500)
                elif self.timer < 41:
                    self.move_arm(self.pos, -30, 1000, 335, 500)
                elif self.timer < 51:
                    self.move_arm(self.pos, -30, 1000, 455, 500)

                elif self.timer <81:
                    self.move_arm([0, 15, 15], 0, 3000, 455, 500)
                else:
                    s = Int16()
                    s.data = 3
                    self.action_p.publish(s)
        else:
            self.pos = None



    def move_arm(self, pos, alp, time=2000, close=90, gamma=500):
        msg1 = Float64MultiArray()
        msg1.data = [float(pos[0]), float(pos[1]), float(pos[2]), float(alp), float(time)]
        msg2 = Int16MultiArray()
        msg2.data = [close, gamma, time]
        
        self.publisher_service1.publish(msg1)
        self.publisher_service2.publish(msg2)





def main():
    rclpy.init()
    node = arm_auto()
    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown(node)


if __name__ == '__main__':
    main()