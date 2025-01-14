#!/usr/bin/env python3

import os
import rospy
import RPi.GPIO as IO
import pigpio
from std_msgs.msg import Float32MultiArray
import serial               #import serial pacakge
from time import sleep
import sys                  #import system package

def GPS_Info():
    global NMEA_buff
    global lat_in_degrees
    global long_in_degrees
    nmea_time = []
    nmea_latitude = []
    nmea_longitude = []
    nmea_time = NMEA_buff[0]                    #extract time from GPGGA string
    nmea_latitude = NMEA_buff[1]                #extract latitude from GPGGA string
    nmea_longitude = NMEA_buff[3]               #extract longitude from GPGGA string
    
    print("NMEA Time: ", nmea_time,'\n')
    print ("NMEA Latitude:", nmea_latitude,"NMEA Longitude:", nmea_longitude,'\n')
    
    lat = float(nmea_latitude)                  #convert string into float for calculation
    longi = float(nmea_longitude)               #convertr string into float for calculation
    
    lat_in_degrees = convert_to_degrees(lat)    #get latitude in degree decimal format
    long_in_degrees = convert_to_degrees(longi) #get longitude in degree decimal format
    
#convert raw NMEA string into degree decimal format   
def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value - int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "%.5f" %(position)
    return position

gpgga_info = "$GPGGA,"
ser = serial.Serial ("/dev/ttyS0")              #Open port with baud rate
GPGGA_buffer = 0
NMEA_buff = 0
lat_in_degrees = 0
long_in_degrees = 0    

if __name__ == '__main__':
    
    rospy.init_node('gps_node')
    rospy.loginfo('gps_node started')
    rate = rospy.Rate(1000)
    global pub
    pub = rospy.Publisher('/gps_data', Float32MultiArray, queue_size=10)

    while not rospy.is_shutdown():

        #READ GPS DATA HERE
        received_data = (str)(ser.readline())                   #read NMEA string received
        GPGGA_data_available = received_data.find(gpgga_info)   #check for NMEA GPGGA string                 
        if (GPGGA_data_available>0):
            GPGGA_buffer = received_data.split("$GPGGA,",1)[1]  #store data coming after "$GPGGA," string 
            NMEA_buff = (GPGGA_buffer.split(','))               #store comma separated data in buffer
            try:
                GPS_Info()                                          #get time, latitude, longitude
    
                print("lat in degrees:", lat_in_degrees," long in degree: ", long_in_degrees, '\n')
                msg = Float32MultiArray()
                msg.data = [lat_in_degrees, long_in_degrees]#ADD GPS DATA HERE
                pub.publish(msg)

            except ValueError:
                print('No signal')
                msg = Float32MultiArray()
                msg.data = [float(0), float(0)]#ADD GPS DATA HERE
                pub.publish(msg)