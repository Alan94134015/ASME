import rclpy
from rclpy.node import Node
from interfaces_pkg.srv import StepMotor
from time import sleep

import RPi.GPIO as GPIO

GPIO.setwarnings(False)  # Disable GPIO warnings
GPIO.setmode(GPIO.BCM)  # or GPIO.BOARD

# Set up the GPIO pins
GPIO.setup(2, GPIO.OUT)
GPIO.setup(3, GPIO.OUT)

class StepMotorService(Node):
    def __init__(self):
        super().__init__('step_motor_node')
        self.srv = self.create_service(StepMotor, '/rotation_service', self.request_callback)

    def request_callback(self, request, response):
        if request.ball_num == 1:
            GPIO.output(2, False)  # Set clockwise rotation

            for i in range(20):
                GPIO.output(3, True)
                sleep(0.001)
                GPIO.output(3, False)
                sleep(0.001)
            
            response.command = "clockwise"
        else:
            GPIO.output(2, True)  # Set counterclockwise rotation

            for i in range(100):
                GPIO.output(3, True)
                sleep(0.001)
                GPIO.output(3, False)
                sleep(0.001)
            response.command = "counterclockwise"
        
        self.get_logger().info(response.command)
        return response
        
    def destroy_node(self):
        super().destroy_node()
        GPIO.output(2, False)
        GPIO.output(3, False)
        GPIO.cleanup() 
    
def main():
    rclpy.init()
    step_motor_service = StepMotorService()
    rclpy.spin(step_motor_service)
    step_motor_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()