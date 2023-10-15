#!/usr/bin/env python

import sys
import socket
import threading

import time
import binascii
from std_msgs.msg import *
import rospy
from sensor_msgs.msg import *
from gps_common.msg import GPSFix


UDP_PORT = 39040
TARGET_IP = '127.0.0.1'
DATA_RATE_HZ = 10
DEBUG = True


server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
address = (TARGET_IP, UDP_PORT)

class GPS_Socket(object):

    def __init__(self,name="GPS_Socket"): 
        rospy.init_node(name)

        self.running = True
        self.hz = rospy.get_param('~hz',10.0)
        self.rate = rospy.Rate(float(self.hz)) 
        self.gps_speed = 0.0

        rospy.Subscriber('/ublox/gpsFix', GPSFix, self.setVehicleLocation, queue_size = 10)
     
    def setVehicleLocation(self, msg):
        self.gps_speed = msg.speed
        #print (self.latitude, self.longitude, self.speed)

    def __del__(self):
        print("Call delete")


    def shutdown(self):
        print("Call Shutdown")


    def run(self):
        while not rospy.is_shutdown():
            msg = None
            try:
                message = bytearray(struct.pack('=BBd', 0x5A, 0xA5, self.gps_speed))
                server_socket.sendto(message, address)

		self.rate.sleep()
            except socket.error as e:
                pass # non-block reads throw exception when data isn't available
                self.rate.sleep()
            except Exception as ex:
                print(ex)
        rospy.loginfo("Stopping Main Thread ...")
        
     
if __name__ == "__main__":
    try:
        node=GPS_Socket()
        node.run()
    except Exception as ex:
        print ex

