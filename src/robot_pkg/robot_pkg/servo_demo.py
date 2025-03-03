import rclpy
from rclpy.node import Node

from gpiozero import Servo
from interfaces_pkg.srv import ServoSrv

from gpiozero import Servo

class MinimalServo(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.servo = Servo(5)
        self.servo.value = self.convert_angle(40)

        self.srv = self.create_service(ServoSrv, '/servo_service', self.trigger_servo)
        self.get_logger().info('伺服馬達連接腸功')
    
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

            self.servo.value = self.convert_angle(angle)    
        except Exception as e:
            pass

        return response
    
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