#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from senseone import BotaSerialSensor, Config  # Import sensor class and config from senseone.py

class SenseOnePublisherROS1:
    """
    A ROS1 node to publish Force/Torque data from a Bota Systems SenseONE sensor.
    """
    def __init__(self):
        """
        Initializes the ROS1 node, sensor, and publisher.
        """
        # --- Sensor Initialization ---
        # Get the serial port from the ROS parameter server, with a default value.
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.sensor = None
        try:
            # Initialize the sensor using the class from senseone.py
            self.sensor = BotaSerialSensor(port, Config.default())
            self.sensor.start()  # Start the sensor's internal reading thread
            rospy.loginfo("Force/Torque sensor initialized on port {}".format(port))
        except Exception as e:
            # Log any errors during sensor initialization and shut down the node.
            rospy.logerr("Failed to initialize sensor: {}".format(e))
            rospy.signal_shutdown("Sensor initialization failed")
            return

        # --- ROS Publisher Initialization ---
        # Create a publisher for the WrenchStamped message.
        # The topic name is 'sense_one/wrench'.
        self.publisher = rospy.Publisher('sense_one/wrench', WrenchStamped, queue_size=10)
        
        # Get the frame_id from the ROS parameter server.
        self.frame_id = rospy.get_param('~frame_id', 'force_torque_sensor')

    def publish_data(self):
        """
        Continuously reads data from the sensor and publishes it.
        This method replaces the timer callback from the ROS2 version.
        """
        # Set the publishing rate (e.g., 100 Hz).
        rate = rospy.Rate(100)
        
        # Main loop that runs until the node is shut down.
        while not rospy.is_shutdown():
            # Create a new WrenchStamped message.
            msg = WrenchStamped()

            # --- Populate the Message ---
            # Update the header with the current ROS time and the configured frame_id.
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id

            # Read the latest sensor data and map it to the wrench message fields.
            # Force components
            msg.wrench.force.x = float(self.sensor.reading.fx)
            msg.wrench.force.y = float(self.sensor.reading.fy)
            msg.wrench.force.z = float(self.sensor.reading.fz)
            
            # Torque components
            msg.wrench.torque.x = float(self.sensor.reading.mx)
            msg.wrench.torque.y = float(self.sensor.reading.my)
            msg.wrench.torque.z = float(self.sensor.reading.mz)

            # --- Publish the Message ---
            self.publisher.publish(msg)
            
            # Sleep to maintain the desired publishing rate.
            rate.sleep()

    def stop(self):
        """
        Cleans up resources by closing the sensor connection.
        """
        rospy.loginfo("Shutting down. Closing sensor port.")
        if self.sensor:
            self.sensor.close()


def main():
    """
    Main function to initialize and run the ROS1 node.
    """
    # Initialize the ROS node with the name 'senseone_ft_publisher'.
    rospy.init_node('senseone_ft_publisher')
    
    # Create an instance of our publisher class.
    node = SenseOnePublisherROS1()
    
    # Register the clean shutdown function.
    rospy.on_shutdown(node.stop)

    try:
        # Call the main publishing loop.
        if not rospy.is_shutdown():
            node.publish_data()
    except rospy.ROSInterruptException:
        # This exception is raised when Ctrl+C is pressed or the node is otherwise shut down.
        rospy.loginfo("Force/torque publisher stopped by user.")


if __name__ == '__main__':
    main()

