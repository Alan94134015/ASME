#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces_pkg.srv import Start
import subprocess

class StartProgramService(Node):
    def __init__(self):
        super().__init__('start_program_service')
        self.srv = self.create_service(Start, '/start_service', self.start_program_callback)

    def start_program_callback(self, request, response):
        try:
            if request.command == 'classfy':
                # 執行另一個 ROS 2 package 的節點
                subprocess.Popen(["ros2", "launch", "camera_pkg", "camera_launch.py"])
                subprocess.Popen(["ros2", "run", "robot_pkg", "step_motor"])
                subprocess.Popen(["ros2", "run", "robot_pkg", "servo"])
                response.success = True
            elif request.command == 'move':
                subprocess.Popen(["ros2", "run", "robot_pkg", "chassis"])

            
        except Exception as e:
            self.get_logger().error(f"Failed to start the program: {e}")
            response.success = False
        return response

def main():
    rclpy.init()
    node = StartProgramService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()