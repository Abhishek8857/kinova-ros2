#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class ToText(Node):
    def __init__(self):
        super().__init__("to_text")
        self.publisher = self.create_publisher(String, "transcription_text", 10)
        self.running = True
        
        input_thread = threading.Thread(target=self.read_input)
        input_thread.daemon = True
        input_thread.start()
    
    def read_input(self):
        while self.running:
            user_input = input("Enter a Command: ")
            if user_input == "exit":
                self.running == False
                self.get_logger().info("Exit command recieved. Shutting down ...")
                rclpy.shutdown()
                break
            
            msg = String()
            msg.data = user_input
            self.publisher.publish(msg=msg)
            self.get_logger().info(f"Published command: {user_input}")
            
            
def main(args=None):
    rclpy.init(args=args)
    node = ToText()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
