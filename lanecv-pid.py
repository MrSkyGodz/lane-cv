import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv
import time

import rospy
from sensor_msgs.msg import Image

class lanecv:
    def __init__(self):

        rospy.init_node("Lane-finder")
        rospy.Subscriber("/counter", Image ,self.callback)    
        rospy.spin()
        self.points = []

    def callback(self, msg):

        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width,-1)
        edge = self.run(img)
        print(self.points)


        cv.imshow("edge",edge)
        cv.waitKey(0)
        cv.destroyAllWindows()

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
                self.points.append([cX,cY])
                cv.circle(img, (cX, cY+four*x), 10, (255, 0, 0), -1)  
                cv.putText(img, str(cX), (cX - 25, cY+four*x - 25),cv.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            except:
                pass
        return img

    def run(self,img):
        edge=self.lane_process(img)
        edge2=self.moment(img,edge)
        return edge2



def main():
    lane = lanecv()

if __name__ == "__main__" :
    main()