#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import keyboard

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.publisher_ = self.create_publisher(String, 'Motor2ControlCMD', 10)
        self.get_logger().info('Keyboard control node started. Press "w" for UP, "s" for DOWN, release for PAUSE.')

        # Biến trạng thái để theo dõi phím
        self.w_pressed = False
        self.s_pressed = False

        # Tạo timer để kiểm tra trạng thái phím
        self.timer = self.create_timer(0.1, self.check_keyboard)

    def check_keyboard(self):
        msg = String()

        # Xử lý phím "w"
        if keyboard.is_pressed('w') and not self.w_pressed:
            msg.data = 'UP'
            self.publisher_.publish(msg)
            self.get_logger().info('Published: UP')
            self.w_pressed = True
        elif not keyboard.is_pressed('w') and self.w_pressed:
            msg.data = 'PAUSE'
            self.publisher_.publish(msg)
            self.get_logger().info('Published: PAUSE')
            self.w_pressed = False

        # Xử lý phím "s"
        if keyboard.is_pressed('s') and not self.s_pressed:
            msg.data = 'DOWN'
            self.publisher_.publish(msg)
            self.get_logger().info('Published: DOWN')
            self.s_pressed = True
        elif not keyboard.is_pressed('s') and self.s_pressed:
            msg.data = 'PAUSE'
            self.publisher_.publish(msg)
            self.get_logger().info('Published: PAUSE')
            self.s_pressed = False

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down keyboard control node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()