from errno import EISDIR
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray, Float64
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
import numpy as np
from geometry_msgs.msg import Twist 

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.listener_callback,
            10)
        self.publisher_service1 = self.create_publisher(Float64MultiArray, '/arm_pos', 10)
        self.publisher_service2 = self.create_publisher(Int16MultiArray, '/arm_close_gamma', 10)
        self.publisher_service3 = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_service4 = self.create_publisher(Int16, '/service', 10)
        self.start_p = self.create_publisher(Int16, '/start', 10)
        self.action_s = self.create_subscription(Int16, '/action', self.action_call, 10)
        self.alp_s = self.create_subscription(Float64, '/alp', self.alp_call, 10)

        # self.timer1 = self.create_timer(0.2, self.publisher1)
        self.timer2 = self.create_timer(0.1, self.publisher2)
        self.timer3 = self.create_timer(0.2, self.c)
        # self.timer = self.create_timer(0.1)

        self.dx = 0
        self.dy = 0
        self.dz = 0
        self.odalp = 0
        self.zz = None
        self.before_start = 2
        self.start_auto = -1
        self.dalp = 0.0001
        self.before_auto = 2
        self.before_dec = 2
        self.before_rot = 2
        
        self.close = 90
        self.beta = 500
        self.odx = 0
        self.ody = 0
        self.odz = 0

        self.x = 0.0
        self.y = 15.0
        self.z = 15.0
        self.alp = 0.0

        self.xx = 0.2
        
        self.flag_auto = -1
        self.flag_model = 0

        self.flag_dec = -1
        self.flag_rot = -1
        self.reset = None

    def action_call(self, msg):
        if msg.data == 3:
            self.flag_auto = -1


    def publisher(self, x):
        msg = Int16()
        msg.data = x
        self.publisher_service4.publish(msg)

    def publisher5(self):
        msg = Int16()
        msg.data = 2
        self.publisher_service4.publish(msg)

    def alp_call(self, msg):
        if np.abs(self.alp - msg.data) > 5:
            self.alp = msg.data
        
    def c(self):
        self.y += self.dy
        self.x += self.dx
        self.z += self.dz
        self.alp += self.dalp

        self.dy = 0
        self.dx = 0
        self.dz = 0
        self.dalp = 0
        print([self.x, self.y, self.z, self.alp, self.beta, self.close])

        print(self.flag_auto, self.flag_model, self.flag_dec, self.flag_rot)



    def publisher1(self, time=150.0):
        msg = Float64MultiArray()
        msg.data = [self.x, self.y, self.z, self.alp, time]
        self.publisher_service1.publish(msg)
        
    

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        # print(msg.axes)
        # print(msg.buttons)
        
        self.ody = msg.axes[1] * self.xx
        self.odx = msg.axes[0] * self.xx
        self.odz = msg.axes[4] * self.xx
        self.odalp = msg.axes[7]
        self.close += (msg.axes[6]) * 10
        self.beta -= msg.axes[5] - 1
        self.beta += msg.axes[2] - 1
        self.reset = msg.buttons[5]
        if self.close > 500:
            self.close = 500
        if self.close <10:
            self.close = 10
        if self.beta > 1000:
            self.beta = 1000
        if self.beta <0:
            self.beta = 0
        self.zz = msg.axes[3]
        if msg.buttons[3] == 1:
            self.flag_model = 1
            self.flag_auto = -1
            self.flag_rot = -1
        elif msg.buttons[1] == 1:
            self.flag_model = -1
            self.flag_auto = -1
            self.flag_rot = -1
        

        if msg.buttons[2] == 0 and self.before_auto == 1 and self.flag_model:
            self.flag_auto = -self.flag_auto
            self.flag_rot = -1
        self.before_auto = msg.buttons[2]

        if msg.buttons[4] == 0 and self.before_dec == 1:
            self.flag_dec = -self.flag_dec
        self.before_dec = msg.buttons[4]

        if msg.buttons[10] == 0 and self.before_start == 1:
            self.start_auto = -self.start_auto
        self.before_start = msg.buttons[10]

        if msg.buttons[12] == 0 and self.before_rot == 1 and self.flag_model == 1 and self.flag_auto == -1:
            self.flag_rot = -self.flag_rot
        self.before_rot = msg.buttons[12]


    def publisher2(self):
        
        # if self.flag_model == 1 and self.flag_auto == 1:
        #     self.publisher(3)
        #     return

        # elif self.flag_model == 1 and self.flag_rot == 1:
        #     self.publisher(2)
        #     return

        # if self.flag_dec == 1:
        #     self.publisher(1)
        
        # if self.flag_model == 1 and self.flag_auto == -1:
        #     msg = Int16MultiArray()
        #     msg.data = [int(self.close), int(self.beta)]
        #     self.publisher_service2.publish(msg)
        #     self.dy += self.ody
        #     self.dx -= self.odx
        #     self.dz += self.odz
        #     self.dalp += self.odalp
        #     self.publisher(-1)
        # # self.dy = 0
        # # self.dx = 0
        # # self.dz = 0
        # # self.dalp = 0
        # elif self.flag_model == -1 and self.flag_auto == -1:
        #     msg2 = Twist()
        #     msg2.linear.x = 5*self.ody
        #     msg2.linear.y = 5*self.odx
        #     msg2.linear.z = 0.0
        #     msg2.angular.x = 0.1*0.0
        #     msg2.angular.y = 0.1*0.0
        #     msg2.angular.z = 0.5*self.zz
        #     self.publisher_service3.publish(msg2)

        # else:
        #     self.publisher(-1)
                 
        msg = Int16()
        msg.data = self.start_auto
        self.start_p.publish(msg)
        if self.start_auto == 1:
            self.x = 0.0
            self.y = 15.0
            self.z = 15.0
            self.alp = 0.0
            self.close = 455
            return
        if self.reset == 1:
            self.x = 0.0
            self.y = 15.0
            self.z = 15.0
            self.alp = 0.0
            self.publisher1(3000.0)
            msg2 = Twist()
            msg2.linear.x = 0.0*0.1
            msg2.linear.y = 0.0*0.1
            msg2.linear.z = 0.0
            msg2.angular.x = 0.1*0.0
            msg2.angular.y = 0.1*0.0
            msg2.angular.z = 0.0*0.1
            self.publisher_service3.publish(msg2)
            return
            
        if self.flag_auto == 1 and self.flag_model == 1:
            # self.close = 455
            self.publisher(3)
            self.x = 0.0
            self.y = 15.0
            self.z = 15.0
            self.alp = 0.0
            self.close = 455
            return
        elif self.flag_auto == 1 and self.flag_model == -1:
            self.publisher(-1)
        elif self.flag_rot == 1:
            self.publisher(2)
            return
        elif self.flag_model == 1 and self.flag_auto == -1:
            msg = Int16MultiArray()
            msg.data = [int(self.close), int(self.beta), 100]
            self.publisher_service2.publish(msg)
            self.dy += self.ody
            self.dx -= self.odx
            self.dz += self.odz
            self.dalp += self.odalp
            self.publisher(-3)
            self.publisher1()
        elif self.flag_model == -1 and self.flag_auto == -1:
            msg2 = Twist()
            msg2.linear.x = self.ody
            msg2.linear.y = self.odx
            msg2.linear.z = 0.0
            msg2.angular.x = 0.1*0.0
            msg2.angular.y = 0.1*0.0
            msg2.angular.z = 0.5*self.zz
            self.publisher_service3.publish(msg2)
            self.publisher(-2)
        elif msg.data == -1:
            self.publisher(-4)

        if self.flag_dec == 1:
            self.publisher(1)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()