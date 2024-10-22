import rclpy
from rclpy.node import Node

from interface_opencv.srv import DetectArUco
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge
import numpy as np
import cv2

Bridge = CvBridge()
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
parameters=cv2.aruco.DetectorParameters()

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(DetectArUco, 'Detect_ArUco',self.detect_callback)
 
    def detect_callback(self,request,response):
        cv_image=self.bridge.imgmsg_to_cv2(request.img, desired_encoding='bgr8')
        detector=cv2.aruco.ArucoDetector(arucoDict,parameters)
        corners, ids, rejected =detector.detectMarkers(cv_image)
        for i in range(len(corners)):
            for j in range(4):
                point = Point32()
                point.x=corners[i][0][j][0]
                point.y=corners[i][0][j][1]
                point.z = 0.0
                response.markers[i].points.append(point)
        
        if ids is None:
            pass
        else:
            response.markers.ids=ids.ravel().tolist()

        output_string="Incoming Request :Image Path:"+request.path
        self.get_logger().info(output_string)
        return response
    
    def main(args=None):
        rclpy.init(args=args)
        minimal_service=MinimalService()
        rclpy.spin(minimal_service)
        rclpy.shutdown()
    
    if __name__=='__main__':
        main()






