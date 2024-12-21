#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from serial import Serial
from std_msgs.msg import String
from pynput import keyboard
import threading

class UartNode(Node):
    def __init__(self):
        super().__init__("uart_node")
        self.publisher = self.create_publisher(String, "uart_data", 10)
        self.serial_port = Serial('/dev/ttyACM1', 115200, timeout=1)
        self.get_logger().info("Serial port opened")
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.subscriber_ = self.create_subscription(String, "uart_data", self.subscriber_callback, 10)

        # Flags for tracking 'w' and 's' key press
        self.w_key_pressed = False
        self.s_key_pressed = False

        # Start listener in a separate thread
        self.listener_thread = threading.Thread(target=self.start_keyboard_listener)
        self.listener_thread.daemon = True
        self.listener_thread.start()

    def subscriber_callback(self, msg):
        # Gelen mesajı işliyoruz
        message = msg.data
        self.get_logger().info(f"Message received: {message}")
        
        try:
            if self.serial_port.is_open:
                self.get_logger().info("Serial port is open")
                self.serial_port.write(message.encode("utf-8"))
        except Exception as e:
            self.get_logger().error(f"Serial port error: {str(e)}")

    def timer_callback(self):
        # Seri port üzerinden gelen verileri okuyup, topic'e yayınlıyoruz
        try:
            if self.serial_port.in_waiting:
                line = self.serial_port.readline().decode("utf-8").rstrip()
                msg = String()
                msg.data = line
                self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error reading from serial port: {str(e)}")

    def send_keyboard_input(self):
        # Send message when a key is pressed
        if self.w_key_pressed:  # 'w' tuşuna basıldığında
            message = "w"
            self.get_logger().info(f"'w' key pressed, sending: {message}")
            
            try:
                if self.serial_port.is_open:
                    self.serial_port.write(message.encode('utf-8'))
                    self.get_logger().info(f"Message '{message}' sent to serial port")
            except Exception as e:
                self.get_logger().error(f"Error sending message: {str(e)}")

        if self.s_key_pressed:  # 's' tuşuna basıldığında
            message = "s"
            self.get_logger().info(f"'s' key pressed, sending: {message}")

            try:
                if self.serial_port.is_open:
                    self.serial_port.write(message.encode('utf-8'))
                    self.get_logger().info(f"Message '{message}' sent to serial port")
            except Exception as e:
                self.get_logger().error(f"Error sending message: {str(e)}")

    def on_press(self, key):
        try:
            if key.char == 'w' and not self.w_key_pressed:  # Tuşa yalnızca ilk kez basıldığında işlem yap
                self.w_key_pressed = True
                self.get_logger().info("'w' key pressed")
            elif key.char == 's' and not self.s_key_pressed:  # Tuşa yalnızca ilk kez basıldığında işlem yap
                self.s_key_pressed = True
                self.get_logger().info("'s' key pressed")
        except AttributeError:
            pass  # Handle special keys

    def on_release(self, key):
        try:
            if key.char == 'w':
                self.w_key_pressed = False
                self.get_logger().info("'w' key released")
            elif key.char == 's':
                self.s_key_pressed = False
                self.get_logger().info("'s' key released")
        except AttributeError:
            pass
    
        if key == keyboard.Key.esc:
            return False  # Stop listener

    def start_keyboard_listener(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()

def main(args=None):
    rclpy.init(args=args)
    node = UartNode()

    # Main loop to listen for keyboard input
    while rclpy.ok():
        node.send_keyboard_input()  # Bu fonksiyon, her zaman tuşları kontrol eder
        rclpy.spin_once(node)  # ROS 2'nin çalışan düğümü kontrol etmesine izin verir

    rclpy.shutdown()

if __name__ == "__main__":
    main()
