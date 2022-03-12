import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv
import time

class lanecv:
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
        h,w=edge.shape
        four=h//4
        edge_list=self.divide(edge)
        for x in range(4):
            try:
                M = cv.moments(edge_list[x])
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cv.circle(img, (cX, cY+four*x), 10, (255, 0, 0), -1)  
                cv.putText(img, str(cX), (cX - 25, cY+four*x - 25),cv.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            except:
                pass
        return img

    def run(self,img):
        edge=self.lane_process(img)
        edge2=self.moment(img,edge)
        return edge2

img = cv.imread('./data/a.jpg')
img2 = img.copy()
start = time.time()
edge=lanecv()
edge2=edge.run(img)

end = time.time()
print(end - start)

cv.imshow("edge",edge2)
cv.waitKey(0)
cv.destroyAllWindows()


