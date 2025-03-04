# 這是開關程式
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from time import sleep

from interfaces_pkg.srv import Start

BUTTON_PIN = 4  # 使用的 GPIO 腳位

class Bottom(Node):
    def __init__(self):
        super().__init__('bottom_node')
        self.cli = self.create_client(Start, '/start_service')

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('等待移動服務開啟')
        
        while GPIO.input(BUTTON_PIN) == GPIO.HIGH:
            sleep(0.1)


        self.get_logger().info("按下按鈕傳送移動要求")
        self.send_request("move")
        self.destroy_node()

    def send_request(self, request):
        req = Start.Request()
        req.command = request
        future = self.cli.call_async(req)
    
    def destroy_node(self):
        super().destroy_node()
        GPIO.cleanup()

def main():
    rclpy.init()
    node = Bottom()
    rclpy.shutdown()

if __name__ == '__main__':
    main()