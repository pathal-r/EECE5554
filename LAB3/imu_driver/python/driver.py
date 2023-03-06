#!/usr/bin/env python
#-*- coding: utf-8 -*-
from imu_driver.msg import Vectornav
import serial
import rospy
import utm
from std_msgs.msg import String
import numpy as np 
import sys
import time
from imu_driver.srv import *

rospy.init_node('mynode')
port = rospy.get_param("~port")

ser = serial.Serial(port=port, baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
pub = rospy.Publisher('imu', Vectornav, queue_size=10)

ser.write("VNWRG,06,0*XX".encode('utf-8'))
ser.write("VNWRG,07,40*XX".encode('utf-8'))
ser.write("$VNWRG,75,2,20,15,0009,000C,0014*XX\r\n".encode())
ser.write("$VNWRG,06,14*XX\r\n".encode())

rate = rospy.Rate(200) # 10hz
msg = Vectornav()



while True: 
    sac = str(ser.readline())
    t = rospy.Time.now()
    if 'YMR' in str(sac):
        pl = sac.split(",")
        print(sac)
        yaw = float(pl[1])*(np.pi/180)
        pitch = float(pl[2])*(np.pi/180)
        roll = float(pl[3])*(np.pi/180)
        mag_x = float(pl[4])
        mag_y = float(pl[5])
        mag_z = float(pl[6])
        accl_x = float(pl[7])
        accl_y = float(pl[8])
        accl_z = float(pl[9])
        angl_x = float(pl[10])
        angl_y = float(pl[11])
        angl_z1 =pl[12]
        angl_z = float(angl_z1[:-8])
        
        convert_to_quaternion = rospy.ServiceProxy('convert_to_quaternion', ConvertToQuaternion)
        quat = convert_to_quaternion(roll, pitch, yaw)
         
        msg.Header.stamp.secs = t.secs
        msg.Header.stamp.nsecs = t.nsecs
        msg.Header.frame_id = "imu1_frame"
        msg.IMU.orientation.x = quat.floatx
        msg.IMU.orientation.y = quat.floaty
        msg.IMU.orientation.z = quat.floatz
        msg.IMU.orientation.w = quat.floatw
        msg.IMU.angular_velocity.x = angl_x
        msg.IMU.angular_velocity.y = angl_y
        msg.IMU.angular_velocity.z = angl_z
        msg.IMU.linear_acceleration.x= accl_x
        msg.IMU.linear_acceleration.y= accl_y
        msg.IMU.linear_acceleration.z= accl_z
        msg.MagField.magnetic_field.x = mag_x
        msg.MagField.magnetic_field.y = mag_y
        msg.MagField.magnetic_field.z = mag_z
        msg.VNYMR = sac
    
        pub.publish(msg)




