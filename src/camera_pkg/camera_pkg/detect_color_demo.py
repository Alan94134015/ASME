import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

from interfaces_pkg.msg import CvImage

colors = {
    "white": ([136, 0, 0], [151, 10, 255]),
    "copper": ([15, 64, 64], [21, 127, 229]),
    "stainless steel": ([15, 16, 36], [26, 62, 182])
}

class CameraDemo(Node):
    def __init__(self):
        super().__init__('detect_color_node')
        self.sub = self.create_subscription(
            CvImage,
            '/image_topic',
            self.image_callback,
            2
        )
        self.period = 0.1
        self.pub = self.create_publisher(CvImage, '/color_detect_topic', 2)
        self.timer = self.create_timer(self.period, self.timer_callback)
        self.bridge = CvBridge()
        self.detect_color = None
        self.pub_msg = CvImage()
    
    def image_callback(self, image_msg):
        try:
            # 将 ROS 图像消息转换为 OpenCV 图像
            roi = self.bridge.imgmsg_to_cv2(image_msg.roi_image, 'bgr8')
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
                    self.detect_color = 'white'
                elif copper > steel:
                    self.detect_color = 'copper'
                else:
                    self.detect_color = 'stainless steel'
                self.pub_msg.detect_color = self.detect_color

                # self.get_logger().info(f'I found {self.detect_color}')

                # final_mask = mask[detected_nonzero_number.index(max(detected_nonzero_number))]
                # cv2.putText(image_msg.roi_image, self.detect_color, (400, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
    
    def timer_callback(self):
        if self.pub_msg:
            self.pub.publish(self.pub_msg)

def main():
    rclpy.init()
    detect_circle = CameraDemo()
    rclpy.spin(detect_circle)
    detect_circle.destroy_node()
    rclpy.shutdown()
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()