#!/usr/bin/env python3

import rospy
import serial
from sensor_msgs.msg import NavSatFix
from ublox_gps import UbloxGps

def gps_node():
    # Initialize the ROS node
    rospy.init_node('gps_node', anonymous=True)

    # Create a publisher to publish GPS data
    pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=50)

    # Set up the serial port and Ublox GPS object
    port = serial.Serial('/dev/ttyACM1', baudrate=115200, timeout=0.1)
    gps = UbloxGps(port)

    # Define the loop rate in Hz
    rate = rospy.Rate(1)  # 1 Hz, adjust as needed

    try:
        rospy.loginfo("Listening for UBX Messages")

        while not rospy.is_shutdown():
            try:
                # Retrieve GPS data
                geo = gps.geo_coords()

                # Create a NavSatFix message
                gps_msg = NavSatFix()

                # Populate the message with GPS data
                gps_msg.latitude = geo.lat
                gps_msg.longitude = geo.lon
                gps_msg.altitude = 0  # Set altitude as NaN if not available
                #gps_msg.heading_of_motion = geo.headMot 

                # For ROS compatibility, use appropriate statuses
                #gps_msg.status.status = NavSatFix.STATUS_FIX
                #gps_msg.status.service = NavSatFix.SERVICE_GPS

                # Publish the message
                pub.publish(gps_msg)

                # Log the data
                rospy.loginfo(f"Longitude: {geo.lon}, Latitude: {geo.lat},Altitude {0}")

            except (ValueError, IOError) as err:
                rospy.logerr(f"Error: {err}")

            # Sleep to maintain loop rate
            rate.sleep()

    finally:
        port.close()

if __name__ == '__main__':
    try:
        gps_node()
    except rospy.ROSInterruptException:
        pass

