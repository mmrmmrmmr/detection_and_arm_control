from ast import In
from platform import node
from tkinter import SEL_LAST
from turtle import pos
import rclpy    
from rclpy.node import Node   
from std_msgs.msg import Int16   
import numpy as np     
from std_msgs.msg import Float64MultiArray, Float64
from std_msgs.msg import Int16MultiArray
from .trans_arm import AK
class trans(Node):
    def __init__(self):
        super().__init__('pos_camera_to_arm')
        self.pos_s = self.create_subscription(Float64MultiArray, '/arm_pos', self.pos_call, 10)
        # self.close_gamma_s = self.create_subscription(Int16MultiArray, '/arm_close_gamma', self.c_g_call, 10)

        # self.stop_s = self.create_subscription(Int16, '/stop', self.stop, 10)

        self.p = self.create_publisher(Int16MultiArray, '/arm_pos_time', 10)
        self.pp = self.create_publisher(Float64, '/alp', 10)

        
        # self.timer = self.create_timer(0.1, self.timer_call)
        self.pos = None
        self.close_gamma = None
        self.time = None

    def pos_call(self, msg):
        self.pos = msg.data
        ans = Int16MultiArray()
        data, alp = AK.setPitchRangeMoving((self.pos[0], self.pos[1], self.pos[2]), self.pos[3], -90, 90, 100)
        ans.data = data + [int(self.pos[4])]
        print(ans.data)
        self.p.publish(ans)
        x = Float64()
        x.data = alp
        self.pp.publish(x)
        

    # def c_g_call(self, msg):
    #     if self.start == 1 and msg.data == 1:
    #         self.timer.reset()
    #         return
    #     elif msg.data == -1 and self.start == 1:
    #         self.timer.cancel()
    #         # if self.action == 3
    #         self.action = None
    #         self.service = None
    #         self.start = None
            
    #     self.start = msg.data

    # def timer_call(self):
    #     msg = Int16()

    #     if self.action is None:
    #     #     msg.data = -1
    #     # elif self.action == 1:
    #         msg.data = 2 
    #     elif self.action == 2:
    #         msg.data = 3           
    #     elif self.action == 3:
    #         print("finish")
    #     self.service_p.publish(msg)

def main():
    rclpy.init()
    node = trans()
    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown(node)


if __name__ == '__main__':
    main()