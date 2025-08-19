from curses import start_color
import pyrealsense2 as rs
import numpy as np
import cv2
from yaml import load
ii = 0

def calcMedian(data):
    # if len(data)%2 == 0:
    #     median = (sorted(data)[int(len(data)/2)] + sorted(data)[int(len(data)/2)+1])/2
    # else:
    median = sorted(data)[int(len(data)/2)]
    return median


class depth_3d_dec:
    def __init__(self, lower, upper, w, h):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, int(w), int(h), rs.format.z16, 30)
        config.enable_stream(rs.stream.color, int(w), int(h), rs.format.bgr8, 30)
        self.w = w
        self.h = h
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        s = self.pipeline.start(config)
        self.lower = lower
        self.upper = upper
        self.center = (int(w/2), int(h/2))
        self.color_image = None
        self.color_image_draw = None
        self.depth_frame = None
        self.depth_intrin = None
        self.c = None
        self.p_2d = np.zeros([3, 2], dtype=int)
        self.p_3d = np.zeros([3, 3], dtype=float)
        self.l = np.zeros(3)
        self.r = 0.03
        self.b_m = np.zeros(3)
        self.b_r = np.zeros(3)
        self.ans_pos = np.zeros(3)
        self.mask = None
        self.arm_pos = None

    def get_image(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        self.depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        self.depth_intrin = self.depth_frame.profile.as_video_stream_profile().intrinsics
        if not self.depth_frame or not color_frame:
            return True
        self.depth_image = np.asanyarray(self.depth_frame.get_data())
        self.color_image = np.asanyarray(color_frame.get_data())
        self.color_image_draw = self.color_image.copy()
        
        # print(self.color_image[100, 100])
        return False

    def get_point_3d(self, point_2d, delta=0):
        d = 3
        dis = [0]
        for i in range(-d, d, 1):
            for j in range(-d, d, 1):
                xx = point_2d[0]+i
                yy = point_2d[1]+j
                xx = max(0, min(xx, self.w-1))
                yy = max(0, min(yy, self.h-1))
                a = self.depth_frame.get_distance(xx, yy)
                if a > 0:
                    dis.append(a)
        diss = calcMedian(dis) + delta
        # print(point_2d, "dis", diss)
        return np.array(rs.rs2_deproject_pixel_to_point(
            self.depth_intrin,
            point_2d,
            diss))


    def draw_text(self, list_text, place):
        if type(list_text) == str:
            cv2.putText(self.color_image_draw, list_text, place, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)
        else:
            a = str()
            for x in list_text:
                a = a + " " + '%.3f'%x
            cv2.putText(self.color_image_draw, a, place, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)

    def dec_d(self):
        # self.pipeline.start(config)
        # print("1")
        if self.get_image():
            return

        if not self.dec_c():
            self.draw_text("No object", (20, 400))
            return False
        self.b_m = self.get_point_3d(self.center)
        self.b_m[2] += self.r/1.5

        # self.b_m = self.get_point_3d(self.center, self.r/1.44)
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
        # cv2.rectangle(self.color_image, (x - 5, y - 5), (x + 5, y + 5), (0, 255, 0), 4)
        # cv2.imshow('deep', self.depth_image)
        x = 0.17
        flag = [0, 0, 0]
        for i in range(len(self.l)):
            if abs(self.l[i] - self.r) < x * self.r:
                flag[i] = 1
                continue
            elif abs(self.l[i] - 1.414*self.r) < x * 1.414 * self.r:
                flag[i] = 2
                continue
            elif abs(self.l[i] - 1.73 * self.r) < x * 1.73 * self.r:
                flag[i] = 3

        judge = flag[0] * flag[1] * flag[2]
        if judge == 8:
            # self.b_r[0] = sum(self.p_3d[:, 0]) / 3
            # self.b_r[1] = sum(self.p_3d[:, 1]) / 3
            # self.b_r[2] = sum(self.p_3d[:, 2]) / 3
            self.b_r = sum(self.p_3d)/3
        elif judge == 6:
            a = flag.index(3)
            if a == 2:
                self.b_r = (self.p_3d[0]+self.p_3d[2])/2
            else:
                self.b_r = (self.p_3d[a] + self.p_3d[a+1]) / 2
        else:
            self.b_r = self.b_m

        ll = self.get_l(self.b_m, self.b_r)

        if ll < 0.01:
            self.ans_pos = self.b_r
        else:
            self.ans_pos = self.b_m
        if self.ans_pos[2] < 0.03:
            self.draw_text("error", (20, 400))
            return False
        # cv2.imshow('real', self.color_image_draw)
        # cv2.waitKey(1)
        else:
            self.draw_text(self.ans_pos, (20, 400))
            pos_camera_raw = np.array(self.ans_pos.tolist() + [1])
            msg = (np.matmul(data, pos_camera_raw[:, None]))[0:3].reshape(3).tolist()
            self.draw_text(msg, (20, 425))
            self.arm_pos = msg
            return True


    def get_3_point(self, x, y):
        cv2.drawContours(self.color_image_draw, self.c, -1, (0, 255, 0), 1)
        M = cv2.moments(self.c)
        self.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        self.c = np.array(self.c)
        self.c = np.squeeze(self.c)
        # print(self.c.shape)
        ans = self.c - [x, y]
        # print(ans.shape)
        ans = ans[:, 0] * ans[:, 0] + ans[:, 1] * ans[:, 1]
        ans = ans.tolist()
        # print(ans.shape)
        index = []  # 用来装位置索引
        for i in range(len(ans)):
            index.append(ans.index(sorted(ans, reverse=True)[i]))
        # print(ans[index[:]])
        num = 0
        for j in range(len(ans)):
            self.p_2d[num] = self.c[index[j], :]
            self.p_3d[num] = self.get_point_3d(self.p_2d[num])
            # print(self.p_3d)
            flag_l = True
            for k in range(num):
                l = self.get_l(self.p_3d[k], self.p_3d[num])
                # print(l)
                if (l < 0.85*self.r) | (l > 2*self.r) :
                    flag_l = False
                    break
            # print(flag_l)
            if flag_l:
                num += 1
            if num == 3:
                break

        # print(self.p_3d)
        self.l[0] = self.get_l(self.p_3d[0], self.p_3d[1])
        self.l[1] = self.get_l(self.p_3d[1], self.p_3d[2])
        self.l[2] = self.get_l(self.p_3d[2], self.p_3d[0])


    def get_l(self, point_3d_1, point_3d_2):
        return np.sqrt(np.square(point_3d_1[0] - point_3d_2[0]) + np.square(point_3d_1[1] - point_3d_2[1]) + np.square(point_3d_1[2] - point_3d_2[2]))

    def dec_c(self):
        hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        # 根据阈值构建掩膜
        mask = cv2.inRange(hsv, self.lower, self.upper)
        
        # cv2.imshow("1", mask)
        # 腐蚀操作
        mask = cv2.erode(mask, None, iterations=2)
        
        # cv2.imshow("2", mask)
        # 膨胀操作，其实先腐蚀再膨胀的效果是开运算，去除噪点
        mask = cv2.dilate(mask, None, iterations=1)
        self.mask = mask
        # cv2.imshow("3", mask)

        # 轮廓检测
        # cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]

        if len(cnts) > 0:
            # 找到面积最大的轮廓
            self.c = max(cnts, key=cv2.contourArea)
            # 确定面积最大的轮廓的外接圆
            ((x, y), radius) = cv2.minEnclosingCircle(self.c)

            if 180 > radius > 10:
                self.get_3_point(x, y)
                cv2.circle(self.color_image_draw, (int(x), int(y)), int(radius), (0, 255, 255), 1)
                cv2.circle(self.color_image_draw, (self.center), 5, (0, 0, 255), -1)
                cv2.circle(self.color_image_draw, (int(x), int(y)), 5, (0, 0, 0), -1)
                # cv2.circle(self.color_image_draw, (self.p_2d[0]), 5, (0, 0, 255), -1)
                # cv2.circle(self.color_image_draw, self.p_2d[1], 5, (0, 255, 0), -1)
                # cv2.circle(self.color_image_draw, self.p_2d[2], 5, (255, 0, 0), -1)

                return True
        return False

def nothing(x):
    pass
import numpy as np
 
def loadtxtmethod(filename):
    data = np.loadtxt(filename,dtype=np.float32,delimiter=',')
    return data
 

d = loadtxtmethod("/home/npu/ros2_ws/src/deep_with_arm/deep_with_arm/color.txt")
print(d)
Lower = d[0]
Upper = d[1]

data = loadtxtmethod("/home/npu/ros2_ws/src/deep_with_arm/deep_with_arm/t.txt")


if __name__ == '__main__':
    # Lower = np.array([35, 43, 46])
    # Upper = np.array([77, 255, 255])

    start = depth_3d_dec(Lower, Upper, int(640), int(480))
#定义窗口名称
    winName='Colors of the rainbow'
    #定义滑动条回调函数，此处pass用作占位语句保持程序结构的完整性

    # img_original=cv2.imread('1.png')
    # #颜色空间的转换
    # img_hsv=cv2.cvtColor(img_original,cv2.COLOR_BGR2HSV)
    #新建窗口
    cv2.namedWindow(winName)
    #新建6个滑动条，表示颜色范围的上下边界，这里滑动条的初始化位置即为黄色的颜色范围
    cv2.createTrackbar('LowerbH',winName,Lower[0],255,nothing)
    cv2.createTrackbar('LowerbS',winName,Lower[1],255,nothing)
    cv2.createTrackbar('LowerbV',winName,Lower[2],255,nothing)
    cv2.createTrackbar('UpperbH',winName,Upper[0],255,nothing)
    cv2.createTrackbar('UpperbS',winName,Upper[1],255,nothing)
    cv2.createTrackbar('UpperbV',winName,Upper[2],255,nothing)

    print("1")
    while True:
        lowerbH=cv2.getTrackbarPos('LowerbH',winName)
        lowerbS=cv2.getTrackbarPos('LowerbS',winName)
        lowerbV=cv2.getTrackbarPos('LowerbV',winName)
        upperbH=cv2.getTrackbarPos('UpperbH',winName)
        upperbS=cv2.getTrackbarPos('UpperbS',winName)
        upperbV=cv2.getTrackbarPos('UpperbV',winName)
        start.lower = np.array([lowerbH,lowerbS,lowerbV])
        start.upper = np.array([upperbH,upperbS,upperbV])
        start.dec_d()
        ii +=1
        cv2.imshow(winName,cv2.bitwise_and(start.color_image,start.color_image,mask=start.mask))

        cv2.imshow("x", start.color_image_draw)
        cv2.waitKey(1)

