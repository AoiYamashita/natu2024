import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
import time
import numpy as np

class Strtegy(Node):
    def __init__(self):
        super().__init__("strategy")
        self.slidepub = self.create_publisher(String,"serial_data",10)
        self.destpub = self.create_publisher(Vector3,"destination",10)
        self.posesub = self.create_subscription(Vector3,"Pose",self.cb,10)
        self.poses = [np.array([1250,1250,np.pi*5/4]),"slide",np.array([1500,1500,np.pi*5/4]),"slide",np.array([1300,600,7/4*np.pi]),np.array([1500,300,7/4*np.pi]),"slide",np.array([1200,900,0.0]),np.array([1400,900,np.pi/2.0]),np.array([1400,900,np.pi]),np.array([1200,900,np.pi]),"slide"]
        self.fase = 0
    def cb(self,data):
        pose = np.array([data.x,data.y,data.z])
        if self.poses[self.fase] == "slide":
            s = String()
            s.data = "Slide"
            self.slidepub.publish(s)
            time.sleep(5)
            self.fase += 1
            return
        delta = self.poses[self.fase] - pose
        if abs(delta[0]) < 30 and abs(delta[1]) < 30 and abs(delta[2]) < np.pi/12:
            self.fase += 1
        else:
            V = Vector3()
            V.x = self.poses[self.fase][0]
            V.y = self.poses[self.fase][1]
            V.z = self.poses[self.fase][2]
            self.destpub.publish(V)

        return
    
def main():
    rclpy.init()
    lz = Strtegy()
    rclpy.spin(lz)
