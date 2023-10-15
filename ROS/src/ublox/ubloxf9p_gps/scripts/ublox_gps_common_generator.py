#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from gps_common.msg import *
from ubloxf9p_msgs.msg import NavPVT, NavVELNED, NavRELPOSNED
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import *
from sensor_msgs.msg import NavSatFix
import time
import math

class ublox_gps_common_generator(object):

    def __init__(self):
        rospy.init_node('ublox_gps_common_generator', anonymous=False)
        
        self.speed = 0.0
        self.track = 0.0
        
        # Subscriber
        rospy.Subscriber('navpvt',NavPVT,self.getUbloxNavPVT)
        rospy.Subscriber('navsatfix',NavSatFix,self.getUbloxNavSatFix)

        # Publisher
        self.gpsFixPub = rospy.Publisher('gps_fix',gps_common.msg.GPSFix,queue_size=10)



    def transformYaw(self,yaw):
        if yaw>=0:
            angle = -yaw+450.0
            while(angle>=360.0):
                angle = angle - 360
        else: 
            angle = -yaw + 90
            while(angle>=360.0):
                angle = angle - 360
        return angle

    def getUbloxNavPVT(self,msg):
        self.track = msg.heading/100000.0
        self.speed = msg.gSpeed /1000.0


    def getUbloxNavSatFix(self,msg):
        convertedMsg = self.convertNavPvtToGPSFix(msg)
        self.gpsFixPub.publish(convertedMsg)


    def getSpeed(self,msg):
        self.speed = msg.data

    def convertNavPvtToGPSFix(self,navSatFix):
        newGPSFixMsg = GPSFix()
        
        try:
            
            newGPSFixMsg.header.stamp = navSatFix.header.stamp
            newGPSFixMsg.header.seq = navSatFix.header.seq
            newGPSFixMsg.header.frame_id = navSatFix.header.frame_id
            
            newGPSFixMsg.status.header.stamp = navSatFix.header.stamp
            newGPSFixMsg.status.status = navSatFix.status.status
            
            newGPSFixMsg.latitude = navSatFix.latitude
            newGPSFixMsg.longitude = navSatFix.longitude
            newGPSFixMsg.altitude = navSatFix.altitude

            newGPSFixMsg.track = self.track
            newGPSFixMsg.speed = self.speed
            
            newGPSFixMsg.position_covariance = navSatFix.position_covariance
            newGPSFixMsg.position_covariance_type = navSatFix.position_covariance_type
            
            # self.debugPub.publish(newGPSFixMsg)
        
        except Exception as ex:
            rospy.logwarn(ex)
        
        return newGPSFixMsg



if __name__ == '__main__':
    try:
        node = ublox_gps_common_generator()
        rospy.spin()
    except Exception as ex: 
       print ex 

