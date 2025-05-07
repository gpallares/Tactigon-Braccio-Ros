#!/usr/bin/env python3
import time
import datetime
from os import path, getcwd
from tactigon_gear import TSkin, TSkinConfig, Hand, GestureConfig, OneFingerGesture
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TactigonTurtlesimController(Node):
    def __init__(self):
        super().__init__('tactigon_turtlesim_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_turtle)

        model_folder = getcwd()

        TSKIN_MAC = "C0:83:43:39:21:57"
        TSKIN_HAND = Hand.RIGHT # Hand.LEFT if hand is left
        TSKIN_NAME = "TSKIN"

        gesture_config = GestureConfig(
            path.join(model_folder, "examples", "gear","model.pickle"), 
            path.join(model_folder, "examples", "gear", "encoder.pickle"),
            "demo",
            datetime.datetime.now(),
            ["up","down","push","pull","twist","circle","swipe_r","swipe_l"]
        )

        self.tskin = TSkin(TSkinConfig(TSKIN_MAC, TSKIN_HAND, TSKIN_NAME, gesture_config))
        self.tskin.start()

        self.get_logger().info(f"Connecting to Tactigon: {self.tskin}")

        while not self.tskin.connected:
            time.sleep(0.1)

        self.get_logger().info("Tactigon connected")

    def control_turtle(self):
        if not self.tskin.connected:
            self.get_logger().info("Reconnecting...")
            time.sleep(0.2)
            return
        

        gesture = self.tskin.gesture

        if gesture:
            print(gesture)
            twist = Twist()

            
            if gesture.gesture == "twist":
                twist.angular.z = 1.0  # Adjust angular velocity 
                self.get_logger().info("Twist gesture detected: Increasing angular velocity")
            elif gesture.gesture == "up":
                twist.linear.x = 1.0  # Adjust linear velocity 
                self.get_logger().info("Up gesture detected: Increasing linear velocity")
            elif gesture.gesture == "down":
                twist.linear.x = -1.0  # Adjust linear velocity 
                self.get_logger().info("Down gesture detected: Decreasing linear velocity")

            self.publisher.publish(twist)

        time.sleep(0.02)


    def shutdown(self):
        if self.tskin.connected:
            self.tskin.terminate()
        self.get_logger().info("Shutting down Tactigon controller")


def main():
    rclpy.init()
    controller = TactigonTurtlesimController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.shutdown()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
