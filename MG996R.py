from gpiozero import Servo
from time import sleep

servo = Servo(4)

while True:
    servo.value = (-1.0)
    sleep(0.5)
    servo.value = (0.0)
    sleep(0.5)
    servo.value = (1.0)
    sleep(0.5)