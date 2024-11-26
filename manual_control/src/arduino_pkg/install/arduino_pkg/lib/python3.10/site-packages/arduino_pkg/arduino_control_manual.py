import rclpy
from rclpy.node import Node
import serial
import pygame
import time


class JoystickServoController(Node):
    def __init__(self):
        super().__init__('joystick_servo_controller')
        self.get_logger().info('Joystick Servo Controller Node Started')

        # Initialize the joystick
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick detected!")
            exit()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Detected joystick: {self.joystick.get_name()}")

        # Connect to Arduino
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 9600)
            time.sleep(2)  # Allow time for Arduino to reset
            self.get_logger().info("Connected to Arduino on /dev/ttyACM0")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            exit()

        # Start controlling servos
        self.control_loop()

    def control_loop(self):
        try:
            while rclpy.ok():
                pygame.event.pump()

                # Read joystick axes (horizontal and vertical)
                horizontal = self.joystick.get_axis(0)  # Axis 0: Horizontal
                vertical = self.joystick.get_axis(1)    # Axis 1: Vertical

                # Map joystick values (-32767 to +32767) to servo Angle values (80 to 100)
                horizontal_angle = self.map_value(horizontal * 32767, -32767, 32767, 80, 100)

                # Map joystick values (-32767 to +32767) to Motor PWM values (1000 to 2000)
                vertical_pwm = self.map_value(vertical * 32767, -32767, 32767, 1000, 2000)

                # Create a single message by concatenating the two values (horizontal_angle, vertical_pwm)
                message = f"{horizontal_angle},{vertical_pwm}\n"

                # Send the combined message to Arduino
                self.arduino.write(message.encode())

                self.get_logger().info(f"Sent to Arduino -> {message.strip()}")
                time.sleep(0.1)  # Adjust for desired refresh rate

        except KeyboardInterrupt:
            self.cleanup()

    def map_value(self, value, in_min, in_max, out_min, out_max):
        """Map a value from one range to another."""
        return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    def cleanup(self):
        self.arduino.close()
        pygame.quit()
        self.get_logger().info("Closed connections and resources.")


def main(args=None):
    rclpy.init(args=args)
    node = JoystickServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
