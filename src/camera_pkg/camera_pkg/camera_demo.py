import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

from interfaces_pkg.srv import Image

# 定義感興趣區域（ROI）的邊界
roi_top = 100
roi_bottom = 200
roi_left = 100
roi_right = 200

class CameraDemo(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.srv = self.create_service(Image, '/image_capture_service', self.image_callback)

        self.video = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.bridge = CvBridge()

    # 定義相機擷取畫面上傳到topic
    def image_callback(self, request, response):
        ret, frame = self.video.read()

        if ret:
            try:
                roi_frame = frame[roi_top:roi_bottom, roi_left:roi_right]
                response.roi_image = self.bridge.cv2_to_imgmsg(roi_frame, "bgr8")
                # self.get_logger().info("成功擷取影像")

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