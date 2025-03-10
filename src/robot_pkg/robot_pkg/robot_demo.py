#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces_pkg.srv import Action
import subprocess

class StartProgramService(Node):
    def __init__(self):
        super().__init__('action_service')
        self.srv = self.create_service(Action, '/action_service', self.start_program_callback)

        self.get_logger().warn("啟動機器服務")

    def start_program_callback(self, request, response):
        try:
            if request.command == 'classfy':
                # 執行另一個 ROS 2 package 的節點
                subprocess.Popen(["ros2", "run", "camera_pkg", "detect_ball"])
                subprocess.Popen(["ros2", "run", "robot_pkg", "step_motor"])
                subprocess.Popen(["ros2", "run", "robot_pkg", "servo"])
                self.get_logger().warn("啟動分類機")
                response.success = True
                
            elif request.command == 'move':
                subprocess.Popen(["ros2", "run", "robot_pkg", "chassis"])
                self.get_logger().warn("啟動底盤")
                response.success = True

            
        except Exception as e:
            self.get_logger().error(f"Failed to start the program: {e}")
            response.success = False

        return response

def main():
    rclpy.init()
    node = StartProgramService()
    subprocess.Popen(["ros2", "run", "camera_pkg", "camera"])
    subprocess.Popen(["ros2", "run", "robot_pkg", "bottom"])
    rclpy.spin(node)    
    rclpy.shutdown()

if __name__ == '__main__':
    main()