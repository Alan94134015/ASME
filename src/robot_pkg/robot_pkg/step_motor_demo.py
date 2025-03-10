import rclpy
from rclpy.node import Node
from interfaces_pkg.srv import StepMotor, Arduino

class StepMotorService(Node):
    def __init__(self):
        super().__init__('step_motor_node')
        self.srv = self.create_service(StepMotor, '/rotation_service', self.request_callback)
        self.cli = self.create_client(Arduino, '/arduino_service')

        self.get_logger().warn('步進馬達連接腸功')
        while not self.cli.wait_for_service(timeout_sec=1):
            self.get_logger().warn('等待arduino服務啟動')
        
        self.req_arduino = Arduino.Request()

    def request_callback(self, request, response):
        self.get_logger().info(f"我偵測到{request.ball_num} 顆球")
        if request.ball_num == 1:
            self.req_arduino.command = 'clockwise'
            response.command = "clockwise"

        elif request.ball_num > 1:
            self.req_arduino.command = 'counterclockwise'
            response.command = "counterclockwise"
        
        elif request.ball_num == 0:
            self.req_arduino.command = 'clockwise'
            response.command = "clockwise"
        
        self.get_logger().info(f"我的運轉方向是{response.command}")

        future = self.send_to_arduino(self.req_arduino.command)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            res_arduino = future.result().msg
        else:
            self.get_logger().error('Service call failed')
            res_arduino.succeed = "Failed"

        self.get_logger().info(res_arduino.msg)        

        return response
    
    def send_to_arduino(self, command):
        future = self.cli.call_async(command)
        self.get_logger().info(f'傳送{command}給arduino')
        return future

def main():
    rclpy.init()
    step_motor_service = StepMotorService()
    rclpy.spin(step_motor_service)
    step_motor_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()