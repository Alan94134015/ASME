import rclpy
from rclpy.node import Node

from interfaces_pkg.srv import  Action, Arduino

class ChassisDemo(Node):
    def __init__(self):
        super().__init__('chassis_node')
        self.cli_to_action = self.create_client(Action, '/action_service')
        self.cli_to_arduino = self.create_client(Arduino, '/arduino_service')

        while not self.cli_to_action.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('等待機器服務啟動...')
        while not self.cli_to_arduino.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('等待arduino服務啟動...')
        
        self.get_logger().info('服務啟動...')

        self.req_action = Action.Request()
        self.req_arduino = Arduino.Request()
    
    # 發出觸發分類機的程式
    def send_req_to_action(self):
        self.req_action.command = "classfy"
        future = self.cli_to_action.call_async(self.req_action)
        return future
    
    def send_req_to_arduino(self):
        self.req_arduino.command = "forward"
        future = self.cli_to_arduino.call_async(self.req_arduino)
        return future

def main():
    rclpy.init()
    chassis = ChassisDemo()

    future = chassis.send_req_to_arduino()
    rclpy.spin_until_future_complete(chassis, future)
    response = future.result().msg

    if response == "Chassis stopped":
        chassis.get_logger().warn("移動完畢")
        future = chassis.send_req_to_action()
        rclpy.spin_until_future_complete(chassis, future)

    chassis.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()