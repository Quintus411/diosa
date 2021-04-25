#!/usr/bin/env python3

""" This program asks a client for data and waits for the response, then sends an ACK. """

# Copyright 2018 Rui Silva.
#
# This file is part of rpsreal/pySX127x, fork of mayeranalytics/pySX127x.
#
# pySX127x is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public
# License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# pySX127x is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
# warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more
# details.
#
# You can be released from the requirements of the license by obtaining a commercial license. Such a license is
# mandatory as soon as you develop commercial activities involving pySX127x without disclosing the source code of your
# own applications, or shipping pySX127x with a closed source product.
#
# You should have received a copy of the GNU General Public License along with pySX127.  If not, see
# <http://www.gnu.org/licenses/>.

import time
import configparser
import rospy
from std_msgs.msg import Int16MultiArray, Float32MultiArray
import sys
import struct
from RF24 import RF24, RF24_PA_LOW, RF24_PA_MAX, RF24_2MBPS


config = configparser.ConfigParser()
config.read('config.ini')

radio = RF24(17, 0)
#parser = LoRaArgumentParser("Lora tester")
global imu_pitch
global imu_roll
global imu_yaw

imu_pitch = 0
imu_roll = 0
imu_yaw = 0

def update_imu_values(msg):
    global imu_pitch
    global imu_roll
    global imu_yaw

    imu_pitch = msg.data[0]
    imu_roll = msg.data[1]
    imu_yaw = msg.data[2]

def process_axes(payload):
    
    throttle = payload[1]
    roll = payload[2]
    yaw = payload[3]
    pitch = payload[4]
    aux1 = payload[5]
    aux2 = payload[6]
    aux3 = payload[7]
    aux4 = payload[8]

    print(
        "Throttle:{} | Roll:{} | Yaw:{} | Pitch:{} | Aux1: {} | Aux2: {} | Aux3: {} | Aux4: {} ".format(
            throttle,
            roll,
            yaw,
            pitch,
            aux1,
            aux2,
            aux3,
            aux4
        ),
        end='\r'
    )

    msg = Int16MultiArray()
    msg.data = [throttle, roll, yaw, pitch, aux1, aux2, aux3, aux4]
    pub.publish(msg)

def send_telemetry():
    radio.stopListening()
    
    buffer = struct.pack("hhhhhhhhh", int(imu_pitch), int(imu_roll), int(imu_yaw), 0, 0, 0, 0, 0, 0)
    success = False
    while not success:
        start_timer = time.monotonic_ns()  # start timer
        result = radio.write(buffer)
        end_timer = time.monotonic_ns()  # end timer
        if not result:
            #print("Transmission failed or timed out")
            #time.sleep(1)
            pass
        else:
            success = True
            #print(
            #    "Transmission successful! Time to Transmit: "
            #    "{} us.".format(
            #        (end_timer - start_timer) / 1000
            #    )
            #)

    radio.startListening()

def await_request():
    radio.startListening()  # put radio in RX mode
    print("awaiting request...")
    while not rospy.is_shutdown():
        has_payload, pipe_number = radio.available_pipe()
        if has_payload:
            # fetch 1 payload from RX FIFO
            length = radio.getDynamicPayloadSize()
            buffer = radio.read(length)
            # use struct.unpack() to convert the buffer into usable data
            # expecting a little endian float, thus the format string "<f"
            # buffer[:4] truncates padded 0s in case payloadSize was not set
            payload = struct.unpack("hhhhhhhhh", buffer)
            # print details about the received packet
            
            command_id = payload[0]
            if command_id == 3: # process axis commands and send telemetry data
                process_axes(payload)
                buffer = struct.pack("hhhhhhhhh", int(imu_pitch), int(imu_roll), int(imu_yaw), 0, 0, 0, 0, 0, 0)  # build a new ACK payload
                radio.writeAckPayload(1, buffer)  # load ACK for next response
                #send_telemetry()
    

if __name__ == '__main__':


    if not radio.begin():
        raise RuntimeError("radio hardware is not responding")
    
    base_address = b"base_station"
    fc_address = b"flight_controller"

    radio.setPALevel(RF24_PA_MAX)#MAX as alternative
    radio.setDataRate(RF24_2MBPS)

    radio.openWritingPipe(fc_address)
    radio.openReadingPipe(1, base_address)

    #radio.payloadSize = 18
    radio.enableAckPayload()

    rospy.init_node('radio_transceiver_node')
    global pub
    pub = rospy.Publisher('/rx_tx', Int16MultiArray, queue_size=10)
    imu_sub = rospy.Subscriber('/imu_readings', Float32MultiArray, update_imu_values)
    rate = rospy.Rate(2)

    await_request()
    