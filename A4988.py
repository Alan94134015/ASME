import RPi.GPIO as GPIO
from time import sleep

# Set the pin numbering mode
GPIO.setmode(GPIO.BCM)  # or GPIO.BOARD

# Set up the GPIO pins
GPIO.setup(2, GPIO.OUT)
GPIO.setup(3, GPIO.OUT)

while True:
    GPIO.output(2, True)

    for i in range(200):
        GPIO.output(3, False)
        sleep(0.001)
        GPIO.output(3, True)
        sleep(0.001)

    GPIO.output(2, False)
    GPIO.output(3, False)
    break