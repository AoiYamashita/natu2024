import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
import numpy as np
#Circle = 1414#(450*pi)[mm]
#hoi-ru 62[mm]

class Operate(Node):
    def __init__(self):
        super().__init__('operater')
        self.sub = self.create_subscription(Vector3,"Pose",self.cb,10)
        self.subP = self.create_subscription(Vector3,"destination",self.setdes,10)
        self.destination = None#np.array([1800-200-45,900,np.pi])
        self.pub = self.create_publisher(String,"serial_data",10)
        self.rate = 0.001
        self.prate = 20000
        self.wrate = 1000
        return
    def cb(self,data):
        if self.destination is None:
            return
        x = data.x
        y = data.y
        theta = data.z
        distance = np.sqrt((self.destination[0]-x)**2+(self.destination[1]-y)**2)
        power = self.rate*distance
        A0 = np.array([x,y])
        A1 = A0 + power*np.array([np.cos(theta),np.sin(theta)])
        B0 = self.destination[0:2]
        B1 = B0 - power*np.array([np.cos(self.destination[2]),np.sin(self.destination[2])])
        t = 0.01
        next = (A0*(1 - t)**3 + 3*A1*t*(1 - t)**2 + 3*B1*t**2*(1 - t) + B0*t**3)
        delta = next - A0
        omega = self.wrate*np.sin(self.destination[2] - theta)
        
        dig = (np.arctan2(delta[1],delta[0])-theta)
        speed = self.prate*np.sqrt(delta[0]**2+delta[1]**2)
        s = String()
        s.data = str("move:{:.3f},{},{}".format(float(dig),int(speed),int(omega)))
        #s.data = str("dig:{:.3f},speed:{},Rspeed:{}".format(float(0),int(0),int(3000)))

        self.pub.publish(s)
        self.get_logger().info("send: %s" % "move:{:.3f},{},{}".format(float(dig),int(speed),int(omega)))
        
        return
    def setdes(self,data):
        self.destination = np.array([data.x,data.y,data.z])

def main():
    rclpy.init()
    lz = Operate()
    rclpy.spin(lz)