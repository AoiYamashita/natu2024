import base64
import simplejpeg
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

import numpy as np
import cv2
from cv2 import aruco
from cv_bridge import CvBridge

ksize = 7

def Get_Color_Point_RBG_Base(img,rgb,under_thresh,upper_thresh):
    minBGR = np.array([rgb[2] - under_thresh, rgb[1] - under_thresh, rgb[0] - under_thresh])
    maxBGR = np.array([rgb[2] + upper_thresh, rgb[1] + upper_thresh, rgb[0] + upper_thresh])

    maskBGR = cv2.inRange(img,minBGR,maxBGR)

    resultRGB = cv2.bitwise_and(img,img, mask = maskBGR)

    gray_rgb = cv2.cvtColor(resultRGB,cv2.COLOR_BGR2GRAY)

    gray_rgb = cv2.medianBlur(gray_rgb,ksize)

    ret, th = cv2.threshold(gray_rgb, np.average(rgb)/3, 255, cv2.THRESH_BINARY)

    return th

def Get_Color_Point_HSV_Base(img,hsv,under_thresh,upper_thresh,rateS=1.0,rateV = 1.0):
    minHSV = np.array([(hsv[0] - under_thresh)/2, hsv[1] - under_thresh*rateS, hsv[2] - under_thresh*rateV],dtype=int)
    maxHSV = np.array([(hsv[0] + upper_thresh)/2, hsv[1] + upper_thresh*rateS, hsv[2] + upper_thresh*rateV],dtype=int)

    maskHSV = cv2.inRange(cv2.cvtColor(img,cv2.COLOR_RGB2HSV),minHSV,maxHSV)

    resultHSV = cv2.bitwise_and(img,img, mask = maskHSV)

    gray_HSV = cv2.cvtColor(resultHSV,cv2.COLOR_BGR2GRAY)

    gray_HSV = cv2.medianBlur(gray_HSV,ksize)

    ret, th = cv2.threshold(gray_HSV, np.min([hsv[0]/2,hsv[1],hsv[0]]), 255, cv2.THRESH_BINARY)

    return th

class birdsEyeViewImage:
    def __init__(self,size = 40):
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_7X7_50)

        self.camera_matrix = np.array([[699.28727872,   0.        , 326.3321078 ],
                                       [  0.        , 698.96451783, 222.24815488],
                                       [  0.        ,   0.        ,   1.        ]],dtype=np.float32)
        self.distortion_coeff = np.array([ 0.16446647 ,-0.88571052 ,-0.0095173   ,0.00431599  ,1.95886302],dtype=np.float32)
        self.size = size
        self.cornerVec = np.array([[size,0],[0,0],[0,size],[size,size]],dtype=np.float32)
        self.M = [None for i in range(10)]
    def transImg(self,img,MarkerPosition = None,cornerVec = None):
        if cornerVec is not None:
            self.cornerVec = cornerVec
        else:
            self.cornerVec = np.array([[self.size,0],[0,0],[0,self.size],[self.size,self.size]],dtype=np.float32)
        if MarkerPosition is None:
            self.MarkerPosition = np.array([[i*53/50.5*(self.size),img.shape[1]/2] for i in range(10)])
            MarkerPosition = self.MarkerPosition
        w,h = img.shape[1],img.shape[0]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(self.camera_matrix,self.distortion_coeff,(w,h),1,(w,h))
        img = cv2.undistort(img, self.camera_matrix,self.distortion_coeff, None, newcameramtx)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(img, self.dictionary)
        #aruco.drawDetectedMarkers(img, corners, ids, (0,255,255))

        corners = np.squeeze(corners)
        ids = np.squeeze(ids)

        Ms = []

        try:            
            M = cv2.getPerspectiveTransform(corners, np.array(self.cornerVec+MarkerPosition[ids],dtype=np.float32))
            Ms.append(M)
        except:
            pass

        # try:
        #     cnt = 0
        #     for i in ids:
        #         M = cv2.getPerspectiveTransform(corners[cnt], np.array(self.cornerVec+MarkerPosition[i],dtype=np.float32))
        #         Ms.append(M)

        #         p = np.array(self.cornerVec+MarkerPosition[i],dtype=int)
        #         for i in range(3):
        #             cv2.line(img,
        #                 pt1=tuple(p[i]),
        #                 pt2=tuple(p[i+1]),
        #                 color=(0, 255, 0),
        #                 thickness=3,
        #                 lineType=cv2.LINE_4,
        #                 shift=0)
        #         p = np.array(corners[cnt],dtype=int)
        #         for i in range(3):
        #             cv2.line(img,
        #                 pt1=tuple(p[i]),
        #                 pt2=tuple(p[i+1]),
        #                 color=(0, 0, 255),
        #                 thickness=3,
        #                 lineType=cv2.LINE_4,
        #                 shift=0)
        #         cnt += 1
        # except:
        #     pass

        #M = sign*ave

        lamda = 0.7

        # try:
        #     for i in range(10):
        #         if self.M[ids[i]] is None:
        #             self.M[ids[i]] = Ms[i]
        #             continue
        #         self.M[ids[i]] *= lamda
        #         self.M[ids[i]] += Ms[i]*(1-lamda)
        # except:
        #     pass

        try:
            if self.M[ids] is None:
                self.M[ids] = Ms[0]
            else:
                self.M[ids] *= lamda
                self.M[ids] += Ms[0]*(1-lamda)
        except:
            pass

        try:
            M = self.M[0]
            w2, h2 = img.shape[1],img.shape[1]#(center+k).max(axis=0).astype(int) + 700 # 鳥瞰画像サイズを拡張（見た目の調整）
            img = cv2.warpPerspective(img, M, (w2,h2) )
        except:
            pass

        #print("-"*50)
        return img
    def GetFieldErea(self,img):
        floor = Get_Color_Point_RBG_Base(img,[170,150,70],50,50)
        floor = cv2.medianBlur(floor,15)
        contours, hierarchy = cv2.findContours(floor, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rects = []
        for i in contours:
            x, y, w, h = cv2.boundingRect(i)
            #cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), cv2.LINE_4)
            rects.append([x,y,w,h])
            #print(x,y,w,h)
        rects = np.array(rects)
        try:
            return int(np.min(rects[:,0])) ,int(np.min(rects[:,1])), int(np.max(rects[:,2]+rects[:,0])), int(np.max(rects[:,3]+rects[:,1]))
        except:
            return None,None,None,None
    def SearchColorLine(self,img,k = 5):
        red = Get_Color_Point_RBG_Base(img,[100,15,15],50,50)
        # black = Get_Color_Point_RBG_Base(img,[30,30,30],30,50)
        blue = Get_Color_Point_RBG_Base(img,[30,30,160],50,50)
        # red = Get_Color_Point_HSV_Base(img,[0,224,178],20,20,rateS=.0,rateV=51.0)
        black = Get_Color_Point_HSV_Base(img,[0,0,0],0,360,rateV=0.25)
        # blue = Get_Color_Point_HSV_Base(img,[240,207,160],20,20,rateS=5.0,rateV=5.0)
        #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        redEdge = cv2.Canny(cv2.medianBlur(red,k), 100, 200, L2gradient=True)
        blackEdge = cv2.Canny(cv2.medianBlur(black,k), 100, 200, L2gradient=True)
        blueEdge = cv2.Canny(cv2.medianBlur(blue,k), 100, 200, L2gradient=True)
        return redEdge,blackEdge,blueEdge
    def SearchLine(self,img,k = 3):
        gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        zeromask = cv2.inRange(gray_img,1,255)

        shiftSize = 5

        newzeromask = zeromask.copy()

        newzeromask[shiftSize:-shiftSize] = cv2.bitwise_and(zeromask[shiftSize*2:],zeromask[:-2*shiftSize])

        newzeromask[:,shiftSize:-shiftSize] = cv2.bitwise_and(zeromask[:,shiftSize*2:],zeromask[:,:-2*shiftSize])

        sobel_x = cv2.Sobel(gray_img, cv2.CV_32F, 1, 0)
        sobel_y = cv2.Sobel(gray_img, cv2.CV_32F, 0, 1)

        sobel_x = cv2.convertScaleAbs(sobel_x, alpha = 0.5)
        sobel_y = cv2.convertScaleAbs(sobel_y, alpha = 0.5)

        sobel = cv2.add(sobel_x, sobel_y)

        sobel = cv2.medianBlur(sobel, k)

        ret, dst = cv2.threshold(sobel ,30,500,cv2.THRESH_BINARY)

        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))

        dst = cv2.erode(dst, kernel)

        Edge = cv2.bitwise_and(dst,newzeromask)
        #cv2.imshow("edgehoge",Edge)

        return Edge
    def SearchEraser(self,img,k = 3):
        img = cv2.medianBlur(img, k)
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV_FULL)
        sv = hsv.copy()
        sv[:,:,0] = 0
        gray = cv2.cvtColor(sv,cv2.COLOR_RGB2GRAY)

        sobel_x = cv2.Sobel(gray, cv2.CV_32F, 1, 0)
        sobel_y = cv2.Sobel(gray, cv2.CV_32F, 0, 1)

        sobel_x = cv2.convertScaleAbs(sobel_x, alpha = 0.5)
        sobel_y = cv2.convertScaleAbs(sobel_y, alpha = 0.5)

        sobel = cv2.add(sobel_x, sobel_y)

        sobel = cv2.medianBlur(sobel, k)

        ret, dst = cv2.threshold(sobel ,30,500,cv2.THRESH_BINARY)

        kernelsize = 3

        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(kernelsize, kernelsize))

        dst_ = cv2.dilate(dst, np.uint8(kernel),iterations=3)
        
        notdst = cv2.bitwise_not(dst_)

        notdst = cv2.medianBlur(notdst, k)

        contours, hierarchy = cv2.findContours(notdst, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        #contours = [x for x in contours if cv2.contourArea(x) > 100]
        lineImg = img.copy()*0
        lineImg = cv2.drawContours(lineImg,contours,-1,(0,225,0),1)
        rectImg = img.copy()*0
        maskImg = np.zeros(img.shape,np.uint8)
        for i in contours:
            hull = cv2.convexHull(i)
            area = cv2.contourArea(hull, True)
            if 10 < area and area < img.shape[0]*img.shape[1]:
                mask = cv2.fillPoly(maskImg.copy(), [i[:,0,:]], (255,255,255), lineType=cv2.LINE_8, shift=0)
                maskdImg = cv2.bitwise_and(hsv.copy(),mask)
                maskedh = maskdImg[:,:,0]
                maskedv = maskdImg[:,:,2]
                h = maskedh[(maskedh > 0) & (maskedv > 100)]
                if h[((255/2 < h) & (h < 2/3*255))].shape[0] >= 0.1*h.shape[0]:
                    rectImg = cv2.fillPoly(rectImg, [i[:,0,:]], (0,255,0), lineType=cv2.LINE_8, shift=0)

        
        return rectImg
    def calcMoment(self,img):
        img = cv2.medianBlur(img,5)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        edge = cv2.Canny(gray,100,255,cv2.THRESH_BINARY)
        ret, dst = cv2.threshold(edge ,10,500,cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(dst, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        objects = []
        for i, contour in enumerate(contours):
            if cv2.contourArea(contour) < 1.5*self.size**2:
                continue
            m = cv2.moments(contour)
            try:
                x,y= m['m10']/m['m00'] , m['m01']/m['m00']
                x, y = round(x), round(y)
                cv2.line(img, (x-5,y-5), (x+5,y+5), (0, 0, 255), 2)
                cv2.line(img, (x+5,y-5), (x-5,y+5), (0, 0, 255), 2)
                objects.append(x,y)
            except:
                pass
        return objects,cv2.cvtColor(edge,cv2.COLOR_GRAY2BGR)
    

class ImgProcess(Node):
    def __init__(self):
        super().__init__("receiver")
        self.br = CvBridge()
        self.subscription = self.create_subscription(Image,"image_raw",self.cb,qos_profile_sensor_data)
        self.publisher = self.create_publisher(Image,"processed",10)
        self.pub = self.create_publisher(String, 'web_image', 10)
        self.bevi = birdsEyeViewImage(25)
        self.cnt = 0
        return
    def cb(self,data):
        self.cnt += 1
        imgdata = self.br.imgmsg_to_cv2(data,'bgr8')

        alpha = 1.2
        beta = 50

        frame= imgdata * alpha
        frame[:,:,:] += beta

        rawimg = np.clip(frame,0,255).astype(np.uint8)

        img = self.bevi.transImg(rawimg)

        x0,y0,x1,y1 = self.bevi.GetFieldErea(img)
        Bird = rawimg.copy()
        print(x0,y0,x1,y1)

        if x0 is not None:
            blank = int(0.15*(x1-x0+y1-y0)/2.0)

            x0 -= blank
            y0 -= blank
            x1 += blank
            y1 += blank
            if x0 < 0:
                x0 = 0
            if y0 < 0:
                y0 = 0

        # try:
        #     Edges = self.bevi.SearchLine(img[y0:y1,x0:x1])
            
        #     points = np.argwhere(Edges > 0)-(self.bevi.MarkerPosition[0]-np.array([x0,y0])+self.bevi.size/2)[::-1]

        #     localmap = np.zeros((251,251))

        #     a = 200/np.max([np.max(points[:,0]) - np.min(points[:,0]),np.max(points[:,1]) - np.min(points[:,1])])

        #     for i in points:
        #         localmap[int(a*(i[1]-np.min(points[:,1]))),int(a*(i[0]-np.min(points[:,0])))] = 1
            
        #     points = np.argwhere(localmap > 0)/a+np.array([np.min(points[:,1]),np.min(points[:,0])])
            
        # except:
        #     pass
        
        try:
            re = self.bevi.SearchEraser(img[y0:y1,x0:x1],5)
            ob,ee = self.bevi.calcMoment(re)
            print(ob)
        except:
            pass

        try:
            img_jpeg = simplejpeg.encode_jpeg(np.array(ee), colorspace = "BGR", quality = 50)
            pub_msg = String()
            pub_msg.data = base64.b64encode(img_jpeg).decode()
            self.pub.publish(pub_msg)
        except:
            pass


def main():
    rclpy.init()
    img_receiver = ImgProcess()
    rclpy.spin(img_receiver)