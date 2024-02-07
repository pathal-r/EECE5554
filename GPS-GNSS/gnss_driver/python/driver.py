#!/usr/bin/env python
from gps_driver.msg import gps_msg
import rospy
import serial
from std_msgs.msg import String
import utm



SENSOR_NAME = "GPS"
rospy.init_node('gps_driver')
serial_port = rospy.get_param('~port',"/dev/ttyUSB0")
serial_baud = rospy.get_param('~baudrate', 4800)
port = serial.Serial(serial_port, serial_baud, timeout=3)
pub = rospy.Publisher('gps' , gps_msg, queue_size= 10)
rospy.logdebug("Initializing" + str(SENSOR_NAME)+ "sensor")
  
rate= rospy.Rate(10)
msg= gps_msg()
  
while True:
   sac = str(port.readline())
   if 'GGA' in str(sac):
     print(sac)
     time = str(sac.split(',')[1])
     lat = str(sac.split(',')[2])
     lon = str(sac.split(',')[4])
     
     sec = float(time[:2])*60*60+float(time[2:4])*60+float(time[4:6])
     nsec = (float(time[6:]))*10e6
     if str(sac.split(',')[3]) == "N":
       x = float(lat[:2])+float(lat[2:])/60
     else :
       x = (float(lat[:2])+float(lat[2:])/60)*-1
     if str(sac.split(',')[5]) == "E":
       y = float(lon[:3])+float(lon[3:])/60
     else:
       y = (float(lon[:3])+float(lon[3:])/60)*-1
     ne = utm.from_latlon(float(x),float(y))
     
     msg.Header.stamp.secs = int(sec)
     msg.Header.stamp.nsecs = int(nsec)
     msg.Header.frame_id = "GPS1_FRAME"
     msg.Latitude = x
     msg.Longitude = y
     msg.Altitude = float(sac.split(',')[9])
     msg.HDOP = float(sac.split(',')[8])
     msg.UTM_easting = ne[0]
     msg.UTM_northing = ne[1]
     msg.UTC = float(time)
     msg.Zone = ne[2]
     msg.Letter= ne[3]
     msg.fix_quality = float(sac.split(',')[6])
     if sac != None:
       pub.publish(msg)


