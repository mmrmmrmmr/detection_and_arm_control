from turtle import pos
import rclpy    
from rclpy.node import Node   
from std_msgs.msg import Float64MultiArray   
import numpy as np     
import numpy as np
from .depth_color_dec import data
 
def loadtxtmethod(filename):
    data = np.loadtxt(filename,dtype=np.float32,delimiter=',')
    return data
 

# data = loadtxtmethod("/home/mmr/Desktop/ddd/ros/src/deep_with_arm/deep_with_arm/color.txt")
class camera_to_arm(Node):
    def __init__(self):
        super().__init__('pos_camera_to_arm')
        self.transform = self.create_subscription(Float64MultiArray, '/pos_in_cam_3d', self.pos_transform, 10)
        self.arm_3d_publisher_raw = self.create_publisher(Float64MultiArray, '/pos_in_arm', 10)
        # self.t = np.array([[-94,137.-82,57.6], [-164.67,182.12,852.56,-705.71], [-86.6,56.6,-502.7,397.8], [0,0,0,1]])
        # self.t = np.array([[111,714.4,65.3,-26.8], [-7.9,1501.8,280,-63.9], [-14,2,3443.8,387.9,-140.1], [0,0,0,1]])
        self.t = data

    def pos_transform(self, msg):
        
        pos_camera_raw = np.array(msg.data[0:3].tolist() + [1])
        msg.data = (np.matmul(self.t, pos_camera_raw[:, None]))[0:3].reshape(3).tolist()
        self.arm_3d_publisher_raw.publish(msg)
        print(msg.data)
    
def main():
    rclpy.init()
    pos_camera_to_arm = camera_to_arm()
    rclpy.spin(pos_camera_to_arm)

    pos_camera_to_arm.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
        
        