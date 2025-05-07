#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from tactigon_msgs.msg import TSkinState
from tactigon_arduino_braccio import Braccio, BraccioConfig, Wrist, Gripper
import time

class GestureSubscriber(Node):
    def __init__(self):
        super().__init__('gesture_subscriber')
        self.subscription = self.create_subscription(
            TSkinState,
            '/tactigon_state',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.triggered = False  # Prevent multiple triggers per gesture
        self.is_grippper_open = True  # Initial state of the gripper


    def listener_callback(self, msg):
        
        if msg.angle_valid:
            
            self.get_logger().info("Angle detected, triggering Braccio!")
            self.triggered = True
            self.trigger_braccio_action(msg)
        

    def trigger_braccio_action(self, msg):
        print("Triggering Braccio action...")

        cfg = BraccioConfig("D1:EF:85:90:07:DE")

        with Braccio(cfg) as braccio:

            print("Connecting...")
            while not braccio.connected:
                time.sleep(0.1)

            print("Connected!")
            
            x = 0
            y = 50
            z = 150
            wrist = Wrist.HORIZONTAL

            if self.is_grippper_open:
                gripper = Gripper.OPEN
                self.is_grippper_open = False
            else:
                gripper = Gripper.CLOSE
                self.is_grippper_open = True

            res, status, move_time = braccio.move(x, y, z, wrist, gripper)
         
            self.get_logger().info(f"Move result: {res}, Status: {status}, Time: {move_time}")

        self.get_logger().info("Action complete. Disconnecting.")

def main(args=None):
    rclpy.init(args=args)
    node = GestureSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
