import base64
import simplejpeg
from KarakuriSlamBaseModule import NDTmatching
from KarakuriSlamBaseModule import ExtendedKalmanFilter
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import numpy as np

class Localization(Node):
    def __init__(self):
        super().__init__("receiver")
        self.subscription = self.create_subscription(Float64MultiArray,"point_data",self.cb,10)
        self.EKF = ExtendedKalmanFilter(0.1,0.1,0.2)
        self.map = np.load("/home/yamashita/natu2024/mapPoints.npy")
        self.pose = np.array([1550,900,np.pi])
        #print((np.array([[-1,0],[0,-1]])@self.map.T).T+np.array([1550,900]))
        matching = NDTmatching(np.array([[10,10]]),self.map,self.pose)
        self.boxSize = matching.Boxsize
        self.center = matching.PointsMin
        pose = matching.optedPose
        self.MapPoints = matching.boxMap.copy()
        print(pose)
        return
    def cb(self,data):
        data = np.array(data.data)  # 受信データを`ndarray`に変換
        matrix = data.reshape((-1, 2))  # 元の形状に戻す
        self.get_logger().info('Received matrix: "%s"' % matrix)

def main():
    rclpy.init()
    lz = Localization()
    rclpy.spin(lz)