import rclpy
from rclpy.node import Node

from gpiozero import Servo
from interfaces_pkg.srv import Servo

class MinimalServo(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.servo = Servo(5)
        self.servo.value = -1

        self.srv = self.create_service(Servo, '/servo_service', self.trigger_servo)
    
    def trigger_servo(self, request, response):
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
        
        self.servo.value = float((angle - 90) / 90)

        return response
        
def main():
    rclpy.init()
    servo = MinimalServo()
    rclpy.spin(servo)
    servo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()