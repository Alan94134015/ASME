# 球數及半徑偵測
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from interfaces_pkg.srv import StepMotor, ServoSrv, Image
from time import sleep

class CameraDemo(Node):
    def __init__(self):
        super().__init__('detect_circle_node')
        self.cli_image = self.create_client(Image, '/image_capture_service')
        self.cli_motor = self.create_client(StepMotor, '/rotation_service')
        
        while not self.cli_image.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待影像服務啟動...')
        
        # 定義傳送資料
        self.req_image = Image.Request()
        self.req_motor = StepMotor.Request()

        self.bridge = CvBridge()
        self.radius = []

    # 發出擷取照片的請求
    def send_request_to_camera(self):
        future = self.cli_image.call_async(self.req_image)
        return future

    # 辨識圓形
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
        try:
            cv2.imshow('detect_circle', roi_image)
            cv2.waitKey(1)
        except:
            pass
    
    # 發出轉動步進馬達的要求
    def send_request_to_stepMotor(self, num):
        self.req_motor.ball_num = num

        future = self.cli_motor.call_async(self.req_motor)
        return future

def main():
    rclpy.init()
    camera = CameraDemo()
    
    while rclpy.ok():
        # 發出擷取照片的請求
        future_image = camera.send_request_to_camera()
        rclpy.spin_until_future_complete(camera, future_image)
        # 辨識球徑大小及數量
        req_image = future_image.result()
        camera.detect_circle(req_image.roi_image)
        # 發出轉動角度的請求
        futur_motor = camera.send_request_to_stepMotor(len(camera.radius))
        rclpy.spin_until_future_complete(camera, futur_motor)

        sleep(1)
        

    camera.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
