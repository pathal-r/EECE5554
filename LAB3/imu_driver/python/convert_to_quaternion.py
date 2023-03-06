#!/usr/bin/env python
import rospy
from imu_driver.srv import ConvertToQuaternion,ConvertToQuaternionResponse 
import numpy as np

def handle_convert_to_quaternion(req):
  qx = np.sin(req.roll/2) * np.cos(req.pitch/2) * np.cos(req.yaw/2) - np.cos(req.roll/2) * np.sin(req.pitch/2) * np.sin(req.yaw/2)
  qy = np.cos(req.roll/2) * np.sin(req.pitch/2) * np.cos(req.yaw/2) + np.sin(req.roll/2) * np.cos(req.pitch/2) * np.sin(req.yaw/2)
  qz = np.cos(req.roll/2) * np.cos(req.pitch/2) * np.sin(req.yaw/2) - np.sin(req.roll/2) * np.sin(req.pitch/2) * np.cos(req.yaw/2)
  qw = np.cos(req.roll/2) * np.cos(req.pitch/2) * np.cos(req.yaw/2) + np.sin(req.roll/2) * np.sin(req.pitch/2) * np.sin(req.yaw/2)
  return ConvertToQuaternionResponse(qx,qy,qz,qw)
  
def convert_to_quaternion():
  rospy.init_node('convert_to_quaternion')
  s = rospy.Service('convert_to_quaternion', ConvertToQuaternion, handle_convert_to_quaternion)
  rospy.spin()
 
if __name__ == "__main__":
  convert_to_quaternion()
