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
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters=cv2.aruco.DetectorParameters()

class MinimalClient(Node):
    def __init__(self):
        super().__init__('client_vid')
        self.cli = self.create_client(DetectArUco, 'detect_aruco')       
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DetectArUco.Request()                                   

    def send_request(self,image_message,img_path, current_frame):
        
        gray = cv2.cvtColor(cv2.imread(img_path), cv2.COLOR_BGR2GRAY)
        image_message = Bridge.cv2_to_imgmsg(gray, encoding="bgr8")
        self.req.img = image_message
        self.req.path = img_path
        self.req.frame=current_frame
        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def logData (numAruco, current_frame, response):
    output_string= "Frame : "+ str(current_frame) + "Number of aruco markers " + str(numAruco) + "\n"
    for i in range(numAruco):
        output_string += "Marker " + str(i+1) + ": id = " + str(response.markers.ids[i]) + " Bounding points = "
        output_string += '(' + ', '.join(map(str, response.markers.points)) + ') ' + '\n'
    return output_string




def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClient()
    minimal_client.send_request()
    vid_path = str(sys.argv[1])
    cap = cv2.VideoCapture(vid_path)
    if cap.isOpened():
        current_frame=0
        while True:
            ret,frame=cap.read()
            if ret :
                if current_frame % 10 == 0 :
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    image_message = Bridge.cv2_to_imgmsg(gray, encoding='bgr8')
                    response = minimal_client.send_request(image_message, vid_path, current_frame)

                    while rclpy.ok():
                        rclpy.spin_once(minimal_client)
                        if minimal_client.future.done():
                            try:
                                response = minimal_client.future.result()
                            except Exception as e:
                                minimal_client.get_logger().info(
                                    'Service call failed %r' % (e,))
                            else:
                                numAruco = len(response.markers.ids)
                                output_string = logData(numAruco,current_frame,response)        
                                minimal_client.get_logger().info(output_string)

                            break
            else:
                break
            current_frame+=1
        cap.release()
    cv2.destroyAllWindows()
                                
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()