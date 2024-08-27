from KarakuriSlamBaseModule import NDTmatching
from KarakuriSlamBaseModule import ExtendedKalmanFilter
import rclpy
import numpy as np
import time
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3


import numpy as np

class Localization(Node):
    def __init__(self):
        super().__init__("receiver")
        self.subscription = self.create_subscription(Float64MultiArray,"point_data",self.cb,10)
        self.sub = self.create_subscription(Vector3,"odm",self.odom,10)
        self.EKF = ExtendedKalmanFilter(0.1,0.1,0.1)
        self.map = np.load("/home/yamashita/natu2024/mapPoints.npy")
        self.pose = np.array([1550,900,np.pi])
        matching = NDTmatching(np.array([[10,10]]),self.map,self.pose)
        self.boxSize = matching.Boxsize
        self.center = matching.PointsMin
        self.MapPoints = matching.boxMap.copy()
        self.Vector = np.zeros(3)
        self.time = time.pref_counter()
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
            #delta = self.pose-matching.optedPose
            #if delta[0:2]@(delta[0:2]).T < 1e6 and delta is not None and matching.minScore < 1.0:
            #    self.pose = matching.optedPose
            self.EKF.Move(np.zeros(3),matching.optedPose,matching.H,0.0)
            print(self.EKF.Pose)
        except:
            pass
        #self.get_logger().info('Received matrix: "%s"' % matrix)
    def odom(self,data):
        dt = time.pref_counter() - self.time
        Vec = np.array([data.x,data.y,data.z])
        self.EKF.Move(Vec,dt)
        self.time = time.pref_counter()
        return

def main():
    rclpy.init()
    lz = Localization()
    rclpy.spin(lz)