import base64
import simplejpeg
from KarakuriSlamBaseModule import ICPmatching
from KarakuriSlamBaseModule import ExtendedKalmanFilter
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import numpy as np

class Localization(Node):
    def __init__(self):
        super().__init__("receiver")
        self.subscription = self.create_subscription(Float64MultiArray,"point_data",self.cb,10)
        return
    def cb(self,data):
        data = np.array(data.data)  # 受信データを`ndarray`に変換
        matrix = data.reshape((-1, 2))  # 元の形状に戻す
        self.get_logger().info('Received matrix: "%s"' % matrix)

def main():
    rclpy.init()
    lz = Localization()
    rclpy.spin(lz)