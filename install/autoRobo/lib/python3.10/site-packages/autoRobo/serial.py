import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import time
import serial
import serial.tools.list_ports

def auto_detect_serial_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if 'STLink' in p.description:
            print('\n'+p.description)
            return p.device
    return None

class mySerial(Node):
    def __init__(self):
        super().__init__("serial")
        
        self.ports = None
        i = 0
        ss = 10
        while self.ports is None:
            print('\rNo STLink device',end='.'*(i%ss)+' '*(ss-i%ss))
            i += 1
            self.ports = auto_detect_serial_port()
            time.sleep(0.05)
        self.serial = serial.Serial(self.ports, 115200, timeout=1)
        print(self.ports)
        self.cnt = 0
        self.sub = self.create_subscription(String,"serial_data",self.cb,10)
        self.pub = self.create_publisher(Vector3, 'odom', 10)
        self.timer = self.create_timer(0.00001,self.read)
    def cb(self,data):
        a = data.data
        self.serial.write(a.encode('utf-8'))
        #self.get_logger().info("send: %s" % a)
    def read(self):
        self.cnt += 1
        try:
            data = self.serial.readline()
            data=data.strip()
            data=data.decode('utf-8')
            if data != "":
                self.cnt += 1
                #self.get_logger().info("serial read: %s" % data)
                #print("serial read {}".format(data))
                x,y,omega = map(float,data.split(","))
                V = Vector3()
                V.x = x
                V.y = y
                V.z = omega
                self.pub.publish(V)
        except:
            #print("no data")
            pass
        

def main():
    rclpy.init()
    serial_ = mySerial()
    rclpy.spin(serial_)
