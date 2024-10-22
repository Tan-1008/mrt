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

Bridge = CvBridge()
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
parameters=cv2.aruco.DetectorParameters()

class MinimalClient(Node):
    def __init__(self):
        super().__init__('client_img')
        self.cli = self.create_client(DetectArUco, 'detect_aruco')       
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DetectArUco.Request()                                   

    def send_request(self):
        img_path = str(sys.argv[1])
        gray = cv2.cvtColor(cv2.imread(img_path), cv2.COLOR_BGR2GRAY)
        image_message = Bridge.cv2_to_imgmsg(gray, encoding="passthrough")
        self.req.img = image_message
        self.req.path = img_path
        self.req.imgorvid = False
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClient()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                numAruco = len(response.imgbounds.ids)
                output_string = "No. of markers = " + str(numAruco) + '\n'
                for i in range(numAruco):
                    output_string += "Marker " + str(i+1) + ": id = " + str(response.markers.ids[i]) + " Bounding points = "
                    output_string += '(' + ', '.join(map(str, response.markers.points)) + ') ' + '\n'
                    

                minimal_client.get_logger().info(output_string)

            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()