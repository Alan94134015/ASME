# robot_demo
# 偵測按紐的程式碼
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import RPi.GPIO as GPIO

class Buttom(Node):
    def __init__(self):
        super().__init__('robot_node')
        GPIO.setmode(GPIO.BCM)  # 或者 GPIO.BOARD
        self.pub = self.create_publisher(String, '/start_service', 2)
        # 设置GPIO引脚
        GPIO.setup(2, GPIO.IN)

    def timer_callback(self):
        if GPIO.input(2) == True:
           self.msg.data = 'run'
           self.get_logger().info('the robot is moving')
        else:
           self.get_logger().info('the robot stop')
           self.msg.data = 'stop'

        self.pub.publish(self.msg)

    def destroy_node(self):
        super().destroy_node()
        GPIO.cleanup()      

def main():
    rclpy.init()
    robot = Buttom()
    while GPIO.input(2):
        rclpy.spin_once(robot)
    robot.get_logger().info('the robot arrived the target place')
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
