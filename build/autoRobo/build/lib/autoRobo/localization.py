from KarakuriSlamBaseModule import NDTmatching
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

import numpy as np

class Localization(Node):
    def __init__(self):
        super().__init__("receiver")
        self.subscription = self.create_subscription(Float64MultiArray,"point_data",self.cb,10)
        self.sub = self.create_subscription(Vector3,"odom",self.odom,10)
        self.EKF = ExtendedKalmanFilter(0.1,0.1,0.1)
        self.map = np.load("/home/yamashita/natu2024/mapPoints.npy")
        self.EKF.Pose = np.array([1550,900,np.pi])
        matching = NDTmatching(np.array([[10,10]]),self.map,self.EKF.Pose)
        self.boxSize = matching.Boxsize
        self.center = matching.PointsMin
        self.MapPoints = matching.boxMap.copy()
        self.Vector = np.zeros(3)
        self.time = time.perf_counter()

        self.pointsPlot = self.create_publisher(String,"Plot",10)
        self.fig,self.ax = plt.subplots()
        self.ax.plot(self.map[:,0],self.map[:,1],lw = 0,marker="o",c='r',markersize=1)
        return
    def cb(self,data):
        data = np.array(data.data)
        points = data.reshape((-1, 2))
        box = 20
        points = np.unique(np.round(points/box)*box, axis=0)
        points = points[(abs(points[:,1]) < 800)]
        points = points[:,::-1]
        try:
            matching = NDTmatching(points,self.MapPoints,self.EKF.Pose.copy(),self.boxSize,self.center.copy())
            print(self.boxSize)        
            #delta = self.pose-matching.optedPose
            #if delta[0:2]@(delta[0:2]).T < 1e6 and delta is not None and matching.minScore < 1.0:
            #    self.pose = matching.optedPose
            ##########
            #plotの表示
            pointsplot, = self.ax.plot(matching.optedPoints[:,0],matching.optedPoints[:,1],lw = 0,marker = 'x',c = "c",markersize=1)
            #pointsplot, = self.ax.plot(points[:,0],points[:,1],lw = 0,marker = 'x',c = "c",markersize=1)
            
            #self.EKF.Move(np.zeros(3),matching.H,matching.optedPose,0.0)
            # Figureをバッファに保存
            buf = BytesIO()
            self.fig.savefig(buf, format='png')
            buf.seek(0)

            # バッファから画像を読み込み、ピクセルデータに変換
            image = np.array(Image.open(buf))[:,:,0:3]
            img_jpeg = simplejpeg.encode_jpeg(np.array(image), colorspace = "RGB", quality = 50)
            pub_msg = String()
            pub_msg.data = base64.b64encode(img_jpeg).decode()
            self.pointsPlot.publish(pub_msg)
        
            pointsplot.remove()
            ########
            print(self.EKF.Pose)
        except:
            pass
        #self.get_logger().info('Received matrix: "%s"' % matrix)
    def odom(self,data):
        dt = time.perf_counter() - self.time
        Vec = np.array([data.x,data.y,data.z])
        self.EKF.Move(Vec,None,None,dt)
        print(self.EKF.Pose)
        self.time = time.perf_counter()
        return

def main():
    rclpy.init()
    lz = Localization()
    rclpy.spin(lz)