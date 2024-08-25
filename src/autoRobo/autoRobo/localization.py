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
        matching = NDTmatching(np.array([[10,10]]),self.map,self.pose)
        self.boxSize = matching.Boxsize
        self.center = matching.PointsMin
        self.MapPoints = matching.boxMap.copy()
        return
    def cb(self,data):
        data = np.array(data.data)
        points = data.reshape((-1, 2))
        box = 30
        points = np.unique(np.round(points/box)*box, axis=0)
        points = points[((abs(points[:,0]) > 150) & (abs(points[:,0]) < 500))| ((abs(points[:,1]) > 150) & (abs(points[:,1]) < 500))]
        points = points[:,::-1]
        try:
            matching = NDTmatching(points,self.MapPoints,self.pose.copy(),self.boxSize,self.center.copy())
            print(self.pose)
            delta = self.pose-matching.optedPose
            if delta[0:2]@(delta[0:2]).T < 1e6 and delta is not None and matching.minScore < 1.0:
                self.pose = matching.optedPose
        except:
            pass
        #self.get_logger().info('Received matrix: "%s"' % matrix)

def main():
    rclpy.init()
    lz = Localization()
    rclpy.spin(lz)