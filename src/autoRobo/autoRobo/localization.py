from KarakuriSlamBaseModule import NDTmatching
from KarakuriSlamBaseModule import ICPmatching
from KarakuriSlamBaseModule import ExtendedKalmanFilter
import rclpy
import numpy as np
import time
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3

#点群データ表示用
import simplejpeg
import matplotlib.pyplot as plt
import numpy as np
from io import BytesIO
from PIL import Image
from std_msgs.msg import String
import base64

#import numpy as np

# def CalcEllpose(S):
#     if (S[0, 0] - S[1, 1])**2 + 4*S[0, 1]**2 < 0 or S[0, 1] == 0:
#         return
#     su2 = ((S[0, 0] + S[1, 1]) + np.sqrt((S[0, 0] - S[1, 1])**2 + 4*S[0, 1]**2)) / 2
#     sv2 = ((S[0, 0] + S[1, 1]) - np.sqrt((S[0, 0] - S[1, 1])**2 + 4*S[0, 1]**2)) / 2
#     slope1 = (su2 - S[0, 0]) / S[0, 1]
#     theta = np.arctan(slope1)
#     if S[0,1] == 0:
#         theta = np.pi/2
#     p = 2.448
#     return p*np.sqrt(su2)*2,p*np.sqrt(sv2)*2,theta

class Localization(Node):
    def __init__(self):
        super().__init__("receiver")
        print("start")
        self.subscription = self.create_subscription(Float64MultiArray,"point_data",self.cb,10)
        self.sub = self.create_subscription(Vector3,"odom",self.odom,10)
        self.EKF = ExtendedKalmanFilter(0.0002,0.0002,0.0002)
        self.map = np.load("/home/yamashita/natu2024/mapPoints.npy")
        self.EKF.Pose = np.array([1550,900,np.pi])
        matching = NDTmatching(np.array([[10,10]]),self.map,self.EKF.Pose)
        print("finish init")
        self.boxSize = matching.Boxsize
        self.center = matching.PointsMin
        self.MapPoints = matching.boxMap.copy()
        self.Vector = np.zeros(3)
        self.PosePub = self.create_publisher(Vector3,"Pose",10)
        #self.Ellpose = self.create_publisher(Vector3,"Ell",10)
        self.time = time.perf_counter()

        self.pointsPlot = self.create_publisher(String,"Plot",10)
        self.fig,self.ax = plt.subplots()
        self.ax.plot(self.map[:,0],self.map[:,1],lw = 0,marker="o",c='r',markersize=1)
        self.localLog = []
        return
    def cb(self,data):
        data = np.array(data.data)
        points = data.reshape((-1, 2))
        points = points[(abs(points[:,1]) < 800)]
        points[:,0] *= -1
        points = points[:,::-1]
        try:
            matching = NDTmatching(points,self.MapPoints,self.EKF.Pose.copy(),self.boxSize,self.center.copy())    
            
            if matching.minScore > 0:
                self.EKF.Move(np.zeros(3),np.linalg.pinv(matching.H),matching.optedPose,0.0)

            ##########
            #plotの表示
            pointsplot, = self.ax.plot(matching.optedPoints[:,0],matching.optedPoints[:,1],lw = 0,marker = 'x',c = "c",markersize=1)
            #pointsplot, = self.ax.plot(points[:,0],points[:,1],lw = 0,marker = 'x',c = "c",markersize=1)
            # Figureをバッファに保存
            buf = BytesIO()
            self.fig.savefig(buf, format='png')
            buf.seek(0)
            # # バッファから画像を読み込み、ピクセルデータに変換
            image = np.array(Image.open(buf))[:,:,0:3]
            img_jpeg = simplejpeg.encode_jpeg(np.array(image), colorspace = "RGB", quality = 50)
            pub_msg = String()
            pub_msg.data = base64.b64encode(img_jpeg).decode()
            self.pointsPlot.publish(pub_msg)
            
            pointsplot.remove()
            ########
            print(matching.optedPose)
            V = Vector3()
            V.x = self.EKF.Pose[0]
            V.y = self.EKF.Pose[1]
            V.z = self.EKF.Pose[2]
            self.PosePub.publish(V)
        except:
            pass
        #self.get_logger().info('Received matrix: "%s"' % matrix)
    def odom(self,data):
        dt = time.perf_counter() - self.time
        Vec = np.array([data.x,data.y,data.z])
        self.EKF.Move(Vec,None,None,dt)
        #print(self.EKF.Pose)
        self.time = time.perf_counter()
        V = Vector3()
        V.x = float(self.EKF.Pose[0])
        V.y = float(self.EKF.Pose[1])
        V.z = float(self.EKF.Pose[2])
        self.PosePub.publish(V)
        # try:
        #     
        #     P = Vector3()
        #     P.x,P.y,P.z = CalcEllpose(self.EKF.Sigma.copy())
        #     self.Ellpose.publish(P)
        # except:
        #     pass
        return

def main():
    rclpy.init()
    lz = Localization()
    rclpy.spin(lz)
