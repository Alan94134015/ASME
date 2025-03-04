# 球數及半徑及顏色偵測
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from interfaces_pkg.srv import StepMotor, ServoSrv, Image
from time import sleep

# 顏色
colors = {
    "white": ([136, 0, 0], [151, 10, 255]),
    "copper": ([15, 64, 64], [21, 127, 229]),
    "stainless steel": ([15, 16, 36], [26, 62, 182])
}

class CameraDemo(Node):
    def __init__(self):
        super().__init__('detect_ball_node')
        self.cli_image = self.create_client(Image, '/image_capture_service')
        self.cli_motor = self.create_client(StepMotor, '/rotation_service')
        self.cli_servo = self.create_client(ServoSrv, 'servo_service')
        
        while not self.cli_image.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('等待影像服務啟動...')
        while not self.cli_motor.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('等待步進馬達服務啟動...')
        while not self.cli_servo.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('等待伺服馬達服務啟動...')
        
        # 定義傳送資料
        self.req_image = Image.Request()
        self.req_motor = StepMotor.Request()
        self.req_servo = ServoSrv.Request()

        self.bridge = CvBridge()
        self.radius = list()
        self.color = str()
        
    # 發出擷取照片的請求
    def send_request_to_camera(self):
        future = self.cli_image.call_async(self.req_image)
        return future
    
    # 發出轉動步進馬達的要求
    def send_request_to_stepMotor(self, num):
        try:
            self.req_motor.ball_num = num

            future = self.cli_motor.call_async(self.req_motor)
            return future
        except Exception as e:
            self.get_logger().error(f'Error in send_request_to_stepMotor: {e}')
    
    # 發送轉動伺服馬達的要求
    def send_request_to_servo(self):
        try:
            self.req_servo.color = self.color
            self.req_servo.radius = int(min(self.radius))
            future = self.cli_servo.call_async(self.req_servo)

            return future
        except Exception as e:
            self.get_logger().error(f'Error in send_request_to_servo: {e}')

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

    # 辨識顏色
    def detect_color(self, image):
        try:
            # 将 ROS 图像消息转换为 OpenCV 图像
            roi = self.bridge.imgmsg_to_cv2(image, 'bgr8')
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            detected_nonzero_number = [0, 0, 0]

            mask = []

            for i, (color_name, (lower, upper)) in enumerate(colors.items()):
                lower = np.array(lower, dtype="uint8")
                upper = np.array(upper, dtype="uint8")
                mask.append(cv2.inRange(hsv, lower, upper))

                if cv2.countNonZero(mask[i]) > 0:
                    detected_nonzero_number[i] = cv2.countNonZero(mask[i])

            if detected_nonzero_number:
                white, copper, steel = detected_nonzero_number[:]
                if copper < 20 and steel < 10:
                    self.color = 'white'
                elif copper > steel:
                    self.color = 'copper'
                else:
                    self.color = 'stainless steel'

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

def main():
    rclpy.init()
    camera = CameraDemo()
    
    while rclpy.ok():
        # 發出擷取照片的請求
        future_image = camera.send_request_to_camera()
        rclpy.spin_until_future_complete(camera, future_image)
        # 辨識球徑大小及數量顏色
        req_image = future_image.result()
        camera.detect_circle(req_image.roi_image)
        camera.detect_color(req_image.roi_image)
        # 發出轉動伺服馬達的請求
        if camera.radius:
            future_servo = camera.send_request_to_servo()
            rclpy.spin_until_future_complete(camera, future_servo)
        # 發出轉動部進馬達的請求
        future_motor = camera.send_request_to_stepMotor(len(camera.radius))
        rclpy.spin_until_future_complete(camera, future_motor)

        sleep(1)

    camera.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
