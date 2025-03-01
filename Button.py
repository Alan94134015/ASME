import RPi.GPIO as GPIO
from time import sleep

# 设置引脚编号模式
GPIO.setmode(GPIO.BCM)  # 或者 GPIO.BOARD

# 设置GPIO引脚
GPIO.setup(2, GPIO.IN)
GPIO.setup(3, GPIO.IN)

while True:
    input_state_2 = GPIO.input(2)
    input_state_3 = GPIO.input(3)
    
    print(f"Pin 2 state: {input_state_2}, Pin 3 state: {input_state_3}")
    sleep(1)