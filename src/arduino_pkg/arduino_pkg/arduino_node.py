import rclpy
from rclpy.node import Node
from interfaces_pkg.srv import Arduino
import serial
from time import sleep

class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_node')
        self.servo_srv = self.create_service(Arduino, '/arduino_service', self.servo_callback)
        self.get_logger().warn('arduino服務啟動')

        self.ser = None  # 追蹤 Serial 連線狀態
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.ser.write(("arduino node 啟動" + "\n").encode())
            self.get_logger().info("成功連接 Arduino")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")

    def servo_callback(self, request, response):
        if self.ser is None:
            self.get_logger().error("Serial connection not established")
            response.msg = "Serial not connected"
            return response

        send_command = request.command
        try:
            # 發送指令並確保傳送結尾有換行符號
            self.ser.write((send_command + "\n").encode())
            self.get_logger().info(f"傳送 {send_command} 給 Arduino")

            if request.command == "forward":
                sleep(70)
                self.get_logger().info("等待移動到目的地")

            # 立即讀取回應並顯示
            response.msg = self.ser.readline().decode().strip()  # 直接讀取並去除兩端空白
            
            # 顯示當前回應
            if response.msg:
                self.get_logger().info(f"Arduino 當前回應: {response.msg}")
            else:
                self.get_logger().info("Arduino 沒有回應任何訊息")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
            response.msg = "Failed"
        
        return response


def main():
    rclpy.init()
    arduino_node = ArduinoNode()
    rclpy.spin(arduino_node)
    arduino_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
