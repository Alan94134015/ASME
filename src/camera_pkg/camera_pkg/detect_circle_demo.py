import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from interfaces_pkg.srv import StepMotor, ServoSrv, Image

class CameraDemo(Node):
    def __init__(self):
        super().__init__('detect_circle_node')
        self.cli_image = self.create_client(Image, '/image_capture_service')
        
        while not self.cli_image.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待影像服務啟動...')

        self.req_image = Image.Request()
        self.bridge = CvBridge()
        self.radius = []

    def send_request_to_camera(self):
        future = self.cli_image.call_async(self.req_image)
        return future

    def detect_circle(self, image):
        if image is None:
            self.get_logger().warn("收到的影像為 None,略過檢測")
            return

        try:
            roi_image = self.bridge.imgmsg_to_cv2(image, "bgr8")

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge 錯誤: {e}")
            return

        gray = cv2.cvtColor(roi_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30, param1=50, param2=30, minRadius=10, maxRadius=70)

        self.radius = []
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv2.circle(roi_image, (x, y), r, (0, 255, 0), 2)
                self.radius.append(r)

        cv2.imshow('detect_circle', roi_image)
        cv2.waitKey(10)

def main():
    rclpy.init()
    detect_circle = CameraDemo()
    
    while rclpy.ok():
        future_image = detect_circle.send_request_to_camera()
        rclpy.spin_until_future_complete(detect_circle, future_image)
        
        req_image = future_image.result()
        detect_circle.detect_circle(req_image.roi_image)

    detect_circle.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
