#!/usr/bin/env python3
# encoding:utf-8
import sys
import time
import numpy as np
from math import sqrt
#import matplotlib.pyplot as plt
from .InverseKinematics import *
# #from Transform import getAngle
# #from mpl_toolkits.mplot3d import Axes3D
# from Board import setBusServoPulse, getBusServoPulse
# import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int16MultiArray
import numpy as np
#机械臂根据逆运动学算出的角度进行移动
ik = IK('arm')
#设置连杆长度
l1 = ik.l1 + 0.75
l4 = ik.l4 - 0.15
ik.setLinkLength(L1=l1, L4=l4)

class ArmIK:
    servo3Range = (0, 1000.0, 0, 240.0) #脉宽， 角度
    servo4Range = (0, 1000.0, 0, 240.0)
    servo5Range = (0, 1000.0, 0, 240.0)
    servo6Range = (0, 1000.0, 0, 240.0)
    y0=0
    y1=0
    y2=0
    y3=0

    def __init__(self):
        self.setServoRange()

    def setServoRange(self, servo3_Range=servo3Range, servo4_Range=servo4Range, servo5_Range=servo5Range, servo6_Range=servo6Range):
        # 适配不同的舵机
        self.servo3Range = servo3_Range
        self.servo4Range = servo4_Range
        self.servo5Range = servo5_Range
        self.servo6Range = servo6_Range
        self.servo3Param = (self.servo3Range[1] - self.servo3Range[0]) / (self.servo3Range[3] - self.servo3Range[2])
        self.servo4Param = (self.servo4Range[1] - self.servo4Range[0]) / (self.servo4Range[3] - self.servo4Range[2])
        self.servo5Param = (self.servo5Range[1] - self.servo5Range[0]) / (self.servo5Range[3] - self.servo5Range[2])
        self.servo6Param = (self.servo6Range[1] - self.servo6Range[0]) / (self.servo6Range[3] - self.servo6Range[2])

    def transformAngelAdaptArm(self, theta3, theta4, theta5, theta6):
        #将逆运动学算出的角度转换为舵机对应的脉宽值
        servo3 = int(round(theta3 * self.servo3Param + (self.servo3Range[1] + self.servo3Range[0])/2))
        if servo3 > self.servo3Range[1] or servo3 < self.servo3Range[0] + 60:
            logger.info('servo3(%s)超出范围(%s, %s)', servo3, self.servo3Range[0] + 60, self.servo3Range[1])
            return False

        servo4 = int(round(theta4 * self.servo4Param + (self.servo4Range[1] + self.servo4Range[0])/2))
        if servo4 > self.servo4Range[1] or servo4 < self.servo4Range[0]:
            logger.info('servo4(%s)超出范围(%s, %s)', servo4, self.servo4Range[0], self.servo4Range[1])
            return False

        servo5 = int(round((self.servo5Range[1] + self.servo5Range[0])/2 - (90.0 - theta5) * self.servo5Param))
        if servo5 > ((self.servo5Range[1] + self.servo5Range[0])/2 + 90*self.servo5Param) or servo5 < ((self.servo5Range[1] + self.servo5Range[0])/2 - 90*self.servo5Param):
            logger.info('servo5(%s)超出范围(%s, %s)', servo5, self.servo5Range[0], self.servo5Range[1])
            return False
        
        if theta6 < -(self.servo6Range[3] - self.servo6Range[2])/2:
            servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2])/2 + (90 + (180 + theta6))) * self.servo6Param))
        else:
            servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2])/2 - (90 - theta6)) * self.servo6Param))
        if servo6 > self.servo6Range[1] or servo6 < self.servo6Range[0]:
            logger.info('servo6(%s)超出范围(%s, %s)', servo6, self.servo6Range[0], self.servo6Range[1])
            return False

        return {"servo3": servo3, "servo4": servo4, "servo5": servo5, "servo6": servo6}

    # def servosMove(self, servos, movetime=None):
    #     #驱动3,4,5,6号舵机转动
    #     time.sleep(0.02)
    #     if movetime is None:
    #         max_d = 0
    #         for i in  range(0, 4):
    #             d = abs(getBusServoPulse(i + 3) - servos[i])
    #             if d > max_d:
    #                 max_d = d
    #         movetime = int(max_d*4)
    #     setBusServoPulse(3, servos[0], movetime)
    #     setBusServoPulse(4, servos[1], movetime)
    #     setBusServoPulse(5, servos[2], movetime)
    #     setBusServoPulse(6, servos[3], movetime)
    #     #setBusServoPulse(6, 300, movetime)
    #     return movetime

    def setPitchRange(self, coordinate_data, alpha1, alpha2, da = 1):
        #给定坐标coordinate_data和俯仰角的范围alpha1，alpha2, 自动在范围内寻找到的合适的解
        #如果无解返回False,否则返回对应舵机角度,俯仰角
        #坐标单位cm， 以元组形式传入，例如(0, 5, 10)
        #da为俯仰角遍历时每次增加的角度
        x, y, z = coordinate_data
        if alpha1 >= alpha2:
            da = -da
        for alpha in np.arange(alpha1, alpha2, da):#遍历求解
            result = ik.getRotationAngle((x, y, z), alpha)
            if result:
                theta3, theta4, theta5, theta6 = result['theta3'], result['theta4'], result['theta5'], result['theta6']
                servos = self.transformAngelAdaptArm(theta3, theta4, theta5, theta6)
                if servos != False:
                    return servos, alpha

        return False

    def setPitchRangeMoving(self, coordinate_data, alpha, alpha1, alpha2, movetime=None):
        #给定坐标coordinate_data和俯仰角alpha,以及俯仰角范围的范围alpha1, alpha2，自动寻找最接近给定俯仰角的解，并转到目标位置
        #如果无解返回False,否则返回舵机角度、俯仰角、运行时间
        #坐标单位cm， 以元组形式传入，例如(0, 5, 10)
        #alpha为给定俯仰角
        #alpha1和alpha2为俯仰角的取值范围
        #movetime为舵机转动时间，单位ms, 如果不给出时间，则自动计算
        x, y, z = coordinate_data
        result1 = self.setPitchRange((x, y, z), alpha, alpha1)
        result2 = self.setPitchRange((x, y, z), alpha, alpha2)
        if result1 != False:
            data = result1
            if result2 != False:
                if abs(result2[1] - alpha) < abs(result1[1] - alpha):
                    data = result2
        else:
            if result2 != False:
                data = result2
            else:
                return False
        servos, alpha = data[0], data[1]

        # movetime = self.servosMove((servos["servo3"], servos["servo4"], servos["servo5"], servos["servo6"]), movetime)
        return [servos['servo3'],servos['servo4'],servos['servo5'],servos['servo6']], alpha
        # return servos, alpha, movetime
        
AK = ArmIK()
# setBusServoPulse(1, 200, 500)
# setBusServoPulse(2, 500, 500)    
# AK.setPitchRangeMoving((0.0, 15.0, 15.0), 0.0, -90, 90, 2000)
       
# def my_subcriber():
#     # ROS节点初始化
#     rospy.init_node('my_subcriber', anonymous=True)
		
#     # 创建一个Subcriber，订阅名为 /my_topic 的topic，注册回调函数 MyInfoCallback()
#     rospy.Subscriber("/arm_pos", Float64MultiArray, MyInfoCallback1)
#     rospy.Subscriber("/arm_close_gamma", Int16MultiArray, MyInfoCallback2)

#     # 循环等待回调函数
#     rospy.spin()


# def MyInfoCallback1(msg):
# 		y = msg.data
# 		if AK.y0 != y[0] or AK.y1 != y[1] or AK.y2 != y[2] or AK.y3 != y[3]:
# 			print(AK.setPitchRangeMoving((y[0], y[1], y[2]), y[3], -90, 90, int(y[4])))
# 		AK.y0=y[0]
# 		AK.y1=y[1]
# 		AK.y2=y[2]
# 		AK.y3=y[3]

# def MyInfoCallback2(msg):
# 	y = msg.data
# 	setBusServoPulse(1, y[0], 100)
# 	setBusServoPulse(2, y[1], 100)  


if __name__ == "__main__":
		
    # my_subcriber()
    print(AK.setPitchRangeMoving((11, -12, 12), -30, -90, 90, int(20)))
    x = AK.setPitchRangeMoving((1, 15, 15), 1, -90, 90, int(20))
    #AK.setPitchRangeMoving((0, 10, 10), -30, -90, 0, 2000)
    #time.sleep(2)
    #AK.drawMoveRange2D(300, 10, 0.2, 10, 30, 0.2, 2.5, -90, 90, 1)
