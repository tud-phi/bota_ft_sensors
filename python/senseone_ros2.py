#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from senseone import BotaSerialSensor, Config  # Import sensor class and config from senseone.py
import time

class SenseOnePublisher(Node):
    def __init__(self):
        super().__init__('senseone_ft_publisher')
        # Create a publisher for the WrenchStamped message on a topic (e.g., "wrench")
        self.publisher_ = self.create_publisher(WrenchStamped, 'sense_one/wrench', 10)
        
        # Initialize the sensor on the serial port (adjust the port as needed)
        port = "/dev/ttyUSB0"
        try:
            self.sensor = BotaSerialSensor(port, Config.default())
            self.sensor.start()  # start the sensor reading thread
            self.get_logger().info("Force/Torque sensor initialized")
        except Exception as e:
            self.get_logger().error("Failed to initialize sensor: {}".format(e))
            rclpy.shutdown()
            return
        
        # Create a timer to publish sensor data at a fixed rate (e.g., 100 Hz)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Create a new WrenchStamped message
        msg = WrenchStamped()

        # Update the header with current ROS time and a frame id (customize as needed)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "force_torque_sensor"

        # Read sensor data and map to the wrench message:
        # Force components
        msg.wrench.force.x = float(self.sensor.reading.fx)
        msg.wrench.force.y = float(self.sensor.reading.fy)
        msg.wrench.force.z = float(self.sensor.reading.fz)
        # Torque components
        msg.wrench.torque.x = float(self.sensor.reading.mx)
        msg.wrench.torque.y = float(self.sensor.reading.my)
        msg.wrench.torque.z = float(self.sensor.reading.mz)

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().debug("Published force/torque data")

def main(args=None):
    rclpy.init(args=args)
    node = SenseOnePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Force/torque publisher stopped by user.")
    finally:
        # Clean up: close sensor port and shutdown ROS2
        node.sensor.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
