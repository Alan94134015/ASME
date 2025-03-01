import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from time import sleep

from interfaces_pkg.msg import CvImage
from interfaces_pkg.srv import StepMotor

# Define the region of interest (ROI) boundaries
roi_top = 100
roi_bottom = 200
roi_left = 100
roi_right = 200

class CameraDemo(Node):
    def __init__(self):
        super().__init__('detect_circle_node')
        self.sub = self.create_subscription(
            CvImage,
            '/image_topic',
            self.image_callback,
            2
        )
        
        self.cli = self.create_client(StepMotor, '/rotation_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = StepMotor.Request()

        self.bridge = CvBridge()
        self.radius = []
    
    def image_callback(self, image_msg):
        try:
            # Convert ROS image message to OpenCV image
            roi_image = self.bridge.imgmsg_to_cv2(image_msg.roi_image, "bgr8")

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
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
    
    def send_request(self, ball_num):
        self.req.ball_num = ball_num
        if len(self.radius) > 1:
            self.req.rev = 100
        else:
            self.req.rev = 20
        return self.cli.call_async(self.req)       
    
def main():
    rclpy.init()
    detect_circle = CameraDemo()
    while rclpy.ok():
        future = detect_circle.send_request(len(detect_circle.radius))
        rclpy.spin_until_future_complete(detect_circle, future)

        sleep(2)

    detect_circle.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()