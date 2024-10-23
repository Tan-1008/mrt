import rclpy
from rclpy.node import Node
import sys

from interface_opencv.srv import DetectArUco
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge
import numpy as np
import cv2 
from cv2 import aruco

Bridge = CvBridge()
arucoDict = aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters=aruco.DetectorParameters()

class MinimalClient(Node):
    def __init__(self):
        super().__init__('client_img')
        self.cli = self.create_client(DetectArUco, 'detect_aruco')       
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DetectArUco.Request()                                   

    def send_request(self):
        img_path = str(sys.argv[1])
        
        image_message = Bridge.cv2_to_imgmsg(cv2.imread(img_path), encoding="bgr8")
        self.req.img = image_message
        self.req.path = img_path
        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    client_img = MinimalClient()
    client_img.send_request()

    while rclpy.ok():
        rclpy.spin_once(client_img)
        if client_img.future.done():
            try:
                response = client_img.future.result()
            except Exception as e:
                client_img.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                numAruco = len(response.markers.ids)
                output_string = "No. of markers = " + str(numAruco) + '\n'
                for i in range(numAruco):
                    output_string += "Marker " + str(i+1) + ": id = " + str(response.markers.ids[i]) + " Bounding points = "
                    output_string += '(' + ', '.join(str, response.markers.bounds[i].points) + ') ' + '\n'
                    

                client_img.get_logger().info(output_string)

            break

    client_img.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()