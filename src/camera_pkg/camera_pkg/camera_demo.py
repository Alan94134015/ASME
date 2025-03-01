import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError

from interfaces_pkg.msg import CvImage

# 定義感興趣區域（ROI）的邊界
roi_top = 100
roi_bottom = 200
roi_left = 100
roi_right = 200

class CameraDemo(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.pub = self.create_publisher(CvImage, '/image_topic', 2)
        self.period = 0.1
        self.timer = self.create_timer(self.period, self.timer_callback)
        self.video = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.bridge = CvBridge()
        self.msg = CvImage()

    # 定義相機擷取畫面上傳到topic
    def timer_callback(self):
        ret, frame = self.video.read()

        if ret:
            frame = cv2.resize(frame, (300, 300))
            try:
                # frame_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                # self.msg.image = frame_msg

                roi_frame = frame[roi_top:roi_bottom, roi_left:roi_right]
                self.msg.roi_image = self.bridge.cv2_to_imgmsg(roi_frame, "bgr8")                

                cv2.rectangle(frame, (roi_left, roi_top), (roi_right, roi_bottom), (255, 0, 0), 2)
                # cv2.imshow('original video', frame)
                # cv2.waitKey(1)  # 添加這行以保持 OpenCV 視窗響應
            except CvBridgeError as e:
                self.get_logger().error(f"CvBridge 錯誤: {e}")
        else:
            self.get_logger().warn("無法捕獲幀")

        self.pub.publish(self.msg)

def main():
    rclpy.init()
    camera = CameraDemo()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()