import rclpy
from rclpy.node import Node

from interfaces_pkg.srv import ServoSrv, Arduino

class MinimalServo(Node):
    def __init__(self):
        super().__init__('servo_node')

        self.srv = self.create_service(ServoSrv, '/servo_service', self.trigger_servo)
        self.cli = self.create_client(Arduino, '/arduino_service')
        while not self.cli.wait_for_service(timeout_sec=1):
            self.get_logger().warn('等待arduino伺服連接')

        self.get_logger().info('伺服馬達連接成功')
        self.req_servo = ServoSrv.Request()
    
    def trigger_servo(self, request, response):
        try:
            # 小球
            if request.color == 'white' and 5 < request.radius < 7:
                angle = 40
            elif request.color == 'copper' and 5 < request.radius < 7:
                angle = 60
            elif request.color == 'stainless steel' and 5 < request.radius < 7:
                angle = 80
            # 大球
            elif request.color == 'white' and 11 < request.radius < 13:
                angle = 100
            elif request.color == 'copper' and 11 < request.radius < 13:
                angle = 120
            elif request.color == 'stainless steel' and 11 < request.radius < 13:
                angle = 140

            future = self.send_req_to_arduino(angle)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                response = future.result().msg
            else:
                self.get_logger().error('Service call failed')
                response.msg = "Failed"

        except Exception as e:
            self.get_logger().error(f"Error in trigger_servo: {e}")
            response.msg = "Failed"

        return response
    
    def send_req_to_arduino(self, angle):
        req = Arduino.Request()
        req.command = f"MOVE {angle}"
        future = self.cli.call_async(req)
        return future

    def convert_angle(self, angle):
        return float((angle - 90) / 90)
        
def main():
    rclpy.init()
    servo = MinimalServo()
    rclpy.spin(servo)
    servo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()