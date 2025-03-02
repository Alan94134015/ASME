import rclpy
from rclpy.node import Node

from gpiozero import Servo
from interfaces_pkg.srv import Servo

class MinimalServo(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.servo = Servo(5)

        self.srv = self.create_service(Servo, '/servo_service', self.trigger_servo)
    
    def trigger_servo(self, request):
        self.servo.value = request.angle

def main():
    rclpy.init()

if __name__ == '__main__':
    main()