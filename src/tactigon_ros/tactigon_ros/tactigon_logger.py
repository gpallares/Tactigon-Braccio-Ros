#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tactigon_msgs.msg import TSkinState as TSkinStateMsg
import csv
from datetime import datetime
import os

class TactigonDataLogger(Node):
    def __init__(self):
        super().__init__('tactigon_data_logger')
        # Subscription to the TSkinStateMsg topic
        self.subscription = self.create_subscription(
            TSkinStateMsg,
            '/tactigon_state',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning
        
        # Initialize CSV file
        self.csv_file = None
        self.writer = None
        self.setup_csv()

    def setup_csv(self):
        """Create a CSV file with headers in a 'logs' directory."""
        # Ensure 'logs' directory exists
        #log_dir = os.getcwd() + "/logs"
        desktop = os.path.join(os.path.join(os.path.expanduser('~')),'Desktop') 
        log_dir = os.path.join(desktop, "logs")
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        # Generate filename with current timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'tactigon_log_{timestamp}.csv'
        self.filepath = os.path.join(log_dir, filename)
        
        # Open CSV file and write headers
        self.csv_file = open(self.filepath, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        header = [
            'timestamp', 'connected', 'battery',
            'selector', 'selector_valid',
            'touchpad_one_finger', 'touchpad_two_finger',
            'touchpad_x_pos', 'touchpad_y_pos', 'touchpad_valid',
            'angle_roll', 'angle_pitch', 'angle_yaw', 'angle_valid',
            'gesture_gesture', 'gesture_probability',
            'gesture_confidence', 'gesture_displacement', 'gesture_valid'
        ]
        self.writer.writerow(header)
        self.get_logger().info(f"Logging data to {self.filepath}")

    def listener_callback(self, msg):
        """Process incoming messages and write to CSV."""
        try:
            # Extract data from the message
            timestamp = datetime.now().isoformat()
            connected = msg.connected
            battery = (msg.battery-3.3)/(4.2-3.3)*100
            selector = msg.selector if msg.selector_valid else ''
            selector_valid = msg.selector_valid

            # Touchpad data (if valid)
            if msg.touchpad_valid:
                touch_one = msg.touchpad.one_finger
                touch_two = msg.touchpad.two_finger
                touch_x = msg.touchpad.x_pos
                touch_y = msg.touchpad.y_pos
            else:
                touch_one = touch_two = touch_x = touch_y = ''

            # Angle data (if valid)
            if msg.angle_valid:
                roll = msg.angle.roll
                pitch = msg.angle.pitch
                yaw = msg.angle.yaw
            else:
                roll = pitch = yaw = ''

            # Gesture data (if valid)
            if msg.gesture_valid:
                gesture = msg.gesture.gesture
                prob = msg.gesture.probability
                conf = msg.gesture.confidence
                disp = msg.gesture.displacement
            else:
                gesture = prob = conf = disp = ''

            # Write row to CSV
            row = [
                timestamp, connected, battery,
                selector, selector_valid,
                touch_one, touch_two, touch_x, touch_y, msg.touchpad_valid,
                roll, pitch, yaw, msg.angle_valid,
                gesture, prob, conf, disp, msg.gesture_valid
            ]
            self.writer.writerow(row)
            self.csv_file.flush()  # Ensure data is written immediately

        except Exception as e:
            self.get_logger().error(f"Error processing message: {str(e)}")

    def shutdown(self):
        """Close the CSV file safely."""
        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info("CSV file closed")

def main(args=None):
    rclpy.init(args=args)
    logger_node = TactigonDataLogger()
    try:
        rclpy.spin(logger_node)
    except KeyboardInterrupt:
        logger_node.get_logger().info("Shutting down logger node...")
    finally:
        logger_node.shutdown()
        logger_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()