import rclpy
from rclpy.node import Node

import RPi.GPIO as GPIO

from interfaces_pkg.srv import Chassis, ServoSrv, Action
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_UP)

class ChassisDemo(Node):
    def __init__(self):
        super().__init__('chassis_node')
        self.cli = self.create_client(Chassis, '/image_capture_to_chassis_service')
        self.cli_to_action = self.create_client(Action, '/action_service')

        while not self.cli_to_action.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('等待機器服務啟動...')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('等待影像服務啟動...')
        
        self.get_logger().info('服務啟動...')

        self.req_image = Chassis.Request()
        self.req_action = Action.Request()
    
    # 發出觸發分類機的程式
    def send_req_to_action(self):
        self.req_action.command = "classfy"
        future = self.cli_to_action.call_async(self.req_action)
        return future

    # 發出擷取照片的請求
    def send_request_to_camera(self):
        future = self.cli.call_async(self.req_image)
        return future

def main():
    rclpy.init()
    chassis = ChassisDemo()
    
    while GPIO.input(4) == GPIO.HIGH:
        future = chassis.send_request_to_camera()
        rclpy.spin_until_future_complete(chassis, future)
        
        result = future.result()
        chassis.get_logger().info(f"left_speed: {result.left_speed}, right_speed: {result.right_speed}.")

        sleep(0.1)

    future = chassis.send_req_to_action()
    rclpy.spin_until_future_complete(chassis, future)
    result = future.result().success

    if result == True:
        chassis.get_logger().warn("移動完畢")

    chassis.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()