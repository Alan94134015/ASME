import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

from interfaces_pkg.srv import Image, Chassis

# 定義感興趣區域（ROI）的邊界
roi_top = 100
roi_bottom = 200
roi_left = 100
roi_right = 200

class CameraDemo(Node):
    def __init__(self):
        super().__init__('camera_node')

        # 定義擷取照片的服務
        self.srv = self.create_service(Image, '/image_capture_service', self.image_callback)
        self.srv_chassis = self.create_service(Chassis, '/image_capture_to_chassis_service', self.callback_to_chassis)

        self.video = cv2.VideoCapture(1, cv2.CAP_V4L2)
        self.bridge = CvBridge()

        self.get_logger().warn("相機已經啟動")

    # 定義相機擷取給底盤 走直線
    def callback_to_chassis(self, request, response):
        ret, frame = self.video.read()

        if ret:
            try:
                self.get_logger().info("成功擷取影像")

                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                lower_detect_color = np.array([9, 64, 120])   # HSV 最低值
                upper_detect_color = np.array([39, 111, 189]) # HSV 最高值

                mask = cv2.inRange(hsv, lower_detect_color, upper_detect_color)

                moments = cv2.moments(mask)
                if moments["m00"] != 0:
                    cx = int(moments["m10"] / moments["m00"])  # x 中心
                    cy = int(moments["m01"] / moments["m00"])  # y 中心

                    response.left_speed = cx
                    response.right_speed = cy
                
                # cv2.imshow('video', mask)
                # cv2.waitKey(1)

            except CvBridgeError as e:
                self.get_logger().error(f"CvBridge 錯誤: {e}")

        return response 
    
    # 定義相機擷取畫面上傳到service
    def image_callback(self, request, response):
        ret, frame = self.video.read()

        if ret:
            try:
                roi_frame = frame[roi_top:roi_bottom, roi_left:roi_right]
                response.roi_image = self.bridge.cv2_to_imgmsg(roi_frame, "bgr8")
                self.get_logger().info("成功擷取影像")

            except CvBridgeError as e:
                self.get_logger().error(f"CvBridge 錯誤: {e}")
        else:
            self.get_logger().warn("無法捕獲影像")

        return response 

def main():
    rclpy.init()
    camera = CameraDemo()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()