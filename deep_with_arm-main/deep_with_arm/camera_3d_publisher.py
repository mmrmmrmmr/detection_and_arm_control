from ast import In
from cgitb import small
from re import X
from socket import NI_NAMEREQD
from tkinter import N
from tkinter.messagebox import NO
import rclpy    
from rclpy.node import Node
from rosidl_runtime_py import set_message_fields   
from std_msgs.msg import Float64MultiArray   
from std_msgs.msg import Int16   
import numpy as np     
import cv2
import pyrealsense2 as rs

from geometry_msgs.msg import Twist 

from .depth_color_dec import Lower, Upper, depth_3d_dec
from .depth_color_dec import calcMedian

class camera_p(Node, depth_3d_dec):
    def __init__(self, lower, upper, w=640, h=480):
        super().__init__('camera_3d_publisher')
        depth_3d_dec.__init__(self, lower, upper, w, h)
        self.publisher_pos_cam_3d = self.create_publisher(Float64MultiArray, '/pos_in_cam_3d', 10)
        self.publisher_service_rot = self.create_publisher(Twist, '/cmd_vel', 10)
        self.action = self.create_publisher(Int16, '/action', 10)
        # self.start = self.create_subscription(Int16, '/service', self.start_service, 20)
        self.start = self.create_subscription(Int16, '/service', self.s, 20)
        
        # self.stop = 1
        self.timer1 = self.create_timer(1, self.publisher1)
        self.timer1.cancel()

        self.timer2 = self.create_timer(0.1, self.pos_3d)
        self.timer_rot = self.create_timer(0.2, self.rot)
        self.timer_rot.cancel()
        # self.timer3 = self.create_timer(3, self.publisher2)
        # self.timer3.cancel()
        # self.if_touch = -1
        self.flag = -1
        # self.if_d = 0
        self.mean_pos = None
        self.num_z = 1


    def rot(self, z, x=0.0):
        msg2 = Twist()
        msg2.linear.x = x
        msg2.linear.y = 0.0
        msg2.linear.z = 0.0
        msg2.angular.x = 0.1*0.0
        msg2.angular.y = 0.1*0.0
        msg2.angular.z = z
        self.publisher_service_rot.publish(msg2)

    # def start_service(self, msg):
    #     print("0")
    #     if (self.flag ^ msg.data) and msg.data == 1:
    #         print("11")
    #         self.if_d = 1
    #         self.flag = msg.data
    #         self.timer1.reset()
    #     elif (self.flag ^ msg.data) and msg.data == -1:
    #         print("00")
    #         self.if_d = 0
    #         self.flag = msg.data
    #         self.cancel_timer()

    def init(self):
        self.mean_pos = None
        self.num_z = 1


    def s(self, msg):
        # if msg.data == 3 and self.flag == not 3:
        #     self.timer1.reset()
        if self.flag != msg.data:
            self.init()
        self.flag = msg.data
        # print(self.if_d)

    def action_p(self, x):
        msg = Int16()
        msg.data = x
        self.action.publish(msg)

    def pos_3d(self):
        # print(self.if_d)
        if self.flag == 3:
            # self.action.publish(-3)
            # if self.dec_d():
            if self.dec_d():
                print("touch")
                self.draw_text("catching", (20, 450))
                if self.mean_pos is None:
                    self.mean_pos = self.ans_pos
                elif self.num_z < 25:
                    self.mean_pos = (self.ans_pos + self.mean_pos) / 2
                    self.num_z += 1
                else:
                    msg = Float64MultiArray()
                    msg.data = self.mean_pos.tolist() + self.p_3d.reshape(9).tolist()
                    # msg.data = [1.1, 0.0]
                    self.publisher_pos_cam_3d.publish(msg)
                    print(msg.data)
                    print(self.mean_pos)
                    
                
            # cv2.imshow("camera", self.color_image_draw)
            # cv2.waitKey(1)
            # return
        
        elif self.flag == 2:
            
            print("rot")
            if self.dec_d():
                if np.abs(self.arm_pos[0]) < 2.5 and self.arm_pos[1] <= 25:
                    print("rot_over")
                    self.draw_text("search over", (20, 450))
                    self.action_p(2)
                    self.rot(0.0)
                if self.arm_pos[1] > 45:
                    self.rot(0.0, -0.1)
                elif self.arm_pos[0] > 3.5:
                    self.rot(-0.1)
                    # self.draw_text("searching", (20, 450))
                elif self.arm_pos[0] < -3.5:
                    self.rot(0.1)
                    # self.draw_text("searching", (20, 450))
                elif self.arm_pos[1] > 35:
                    self.rot(0.0, -0.05)
                elif self.arm_pos[1] < 12:
                    self.rot(0.0, 0.05)
                
                # else:
                #     print("rot_over")
                #     self.draw_text("search over", (20, 450))
                #     self.action_p(2)
                #     self.rot(0.0)
                # else:
                #     self.rot(0.15)
                self.draw_text("searching", (20, 450))
            else:
                # self.action.publish(-2)
                self.rot(0.25)
                self.draw_text("searching", (20, 450))
            # self.flag_z = -1
            # cv2.imshow("camera", self.color_image_draw)
            # cv2.waitKey(1)
            # return

        elif self.flag == 1:
            if self.dec_d():
                print(self.ans_pos)
            
        else:
            self.get_image()
            self.draw_text("just watching", (20, 400))

        # if self.flag_z == -1 and self.num_z != 1 and self.flag != 3:
        #     self.num_z = -1
        #     self.flag_z = -1    
        cv2.imshow("camera", self.color_image_draw)
        cv2.waitKey(1)
        # print(self.p_3d[0])
        # print([1, 2])

    def publisher1(self):
        msg = Float64MultiArray()
        # print
        # msg.data = self.mean_pos.tolist() + self.p_3d.reshape(9).tolist()
        msg.data = self.ans_pos.tolist() + self.p_3d.reshape(9).tolist()
        # msg.data = [1.1, 0.0]
        self.publisher_pos_cam_3d.publish(msg)
        print(msg.data)
        print(self.mean_pos)
        # if self.num_z > 5:
        #     self.cancel_timer()
        #     self.flag_z == 1
        # self.num_z += 1


def main():
    rclpy.init()
    # lower = np.array([55, 110, 60])
    # upper = np.array([85, 255, 110])
    camera_3d_publisher = camera_p(Lower, Upper)
    rclpy.spin(camera_3d_publisher)

    camera_3d_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()


