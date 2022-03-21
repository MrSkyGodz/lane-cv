import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv
import time

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class lanecv:
    def __init__(self):
        rospy.init_node("Lane-finder")
        self.bridge = CvBridge()

        self.pub_edge = rospy.Publisher("/edge_right", Image, queue_size=1)
        # self.pub_pid = rospy.Publisher("/pid", Float32, queue_size=1)
        self.pub_pid = rospy.Publisher('/cmd_vel', Twist, queue_size=1)    

        rospy.Subscriber("/camera/right/image_raw", Image ,self.callback)    

        self.points = []
        self.mid = 0.7
        self.direction = 0
        self.P = 3
        self.I = 0
        self.D = 0

        rospy.spin()
       
    def callback(self, msg):

        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width,-1)
        
        
        edge = self.run(img)

        self.pid()
        edge_msg = Image()
        edge_msg = self.bridge.cv2_to_imgmsg(edge, encoding="passthrough")
        self.pub_edge.publish(edge_msg)

        # direction_msg = Float32()
        # direction_msg.data = self.direction

        direction_msg = Twist()
        direction_msg.angular.z = self.direction
        direction_msg.linear.x = 0.2

        self.pub_pid.publish(direction_msg)

        #print(self.points)

        # cv.imshow("edge",edge)
        # cv.waitKey(0)
        # cv.destroyAllWindows()

    def lane_process(self,img):
        img=cv.cvtColor(img,cv.COLOR_BGR2GRAY)
        img=cv.bilateralFilter(img,12,150,150)
        img=cv.equalizeHist(img)
        ret,edge = cv.threshold(img,250,255,cv.THRESH_TOZERO)
        return edge

    def divide(self,img):
        h,w=img.shape
        four=h//4
        divided=[]
        divided.append(img[:four, :])
        divided.append(img[four:2*four, :])
        divided.append(img[2*four:3*four, :])
        divided.append(img[3*four:, :])
        cv.waitKey(0)
        return divided

    def moment(self,img,edge):
        self.points = []
        h,w=edge.shape
        four=h//4 
        edge_list=self.divide(edge)
        for x in range(4):
            try:
                M = cv.moments(edge_list[x])
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                self.points.append([x,cX/w,cY/h])
                cv.circle(img, (cX, cY+four*x), 10, (255, 0, 0), -1)  
                cv.putText(img, str(cX), (cX - 25, cY+four*x - 25),cv.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            except:
                pass
        return img

    def pid(self):
        e = self.mid - self.points[0][1]
        self.direction = e*self.P
        #print(self.direction)

    def run(self,img):
        edge=self.lane_process(img)
        edge2=self.moment(img,edge)
        return edge2



def main():
    lane = lanecv()

if __name__ == "__main__" :
    main()