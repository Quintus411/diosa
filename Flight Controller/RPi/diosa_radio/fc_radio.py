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
from SX127x.LoRa import *
import configparser
import rospy
from std_msgs.msg import Int16MultiArray
import sys
import struct
from RF24 import RF24, RF24_PA_LOW


config = configparser.ConfigParser()
config.read('config.ini')

radio = RF24(17, 0)
#parser = LoRaArgumentParser("Lora tester")

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
        )
    )

    msg = Int16MultiArray()
    msg.data = [throttle, roll, yaw, pitch, aux1, aux2, aux3, aux4]
    pub.publish(msg)

def send_telemetry():
    radio.stopListening()
    
    # use struct.pack() to packet your data into the payload
    accelerometer = 420
    gyroscope = 421
    barometer = 422
    buffer = struct.pack("sssssssss", accelerometer, gyroscope, barometer, 0, 0, 0, 0, 0, 0)
    success = False
    while not success:
        start_timer = time.monotonic_ns()  # start timer
        result = radio.write(buffer)
        end_timer = time.monotonic_ns()  # end timer
        if not result:
            print("Transmission failed or timed out")
            time.sleep(1)
        else:
            success = True
            print(
                "Transmission successful! Time to Transmit: "
                "{} us.".format(
                    (end_timer - start_timer) / 1000
                )
            )

    radio.startListening()

def await_request():
    radio.startListening()  # put radio in RX mode
    print("awaiting request...")
    while True:
        has_payload, pipe_number = radio.available_pipe()
        if has_payload:
            # fetch 1 payload from RX FIFO
            #length = radio.getDynamicPayloadSize()
            buffer = radio.read(radio.payloadSize)
            # use struct.unpack() to convert the buffer into usable data
            # expecting a little endian float, thus the format string "<f"
            # buffer[:4] truncates padded 0s in case payloadSize was not set
            payload = struct.unpack("sssssssss", buffer)
            # print details about the received packet
            
            command_id = payload[0]
            if command_id == 3: # process axis commands and send telemetry data
                process_axes(payload)
                send_telemetry()


class mylora(LoRa):

    def on_rx_done(self):
        BOARD.led_on()
        #print("\nRxDone")
        self.clear_irq_flags(RxDone=1)
        payload = self.read_payload(nocheck=True)
        print ("Receive: ")
        print(payload)
        sender = payload[0]
        recipient = payload[1]
        message_type_A = payload[2]
        message_type_B = payload[3]
        BOARD.led_off()
        if (recipient == self.fc_id or recipient == 255):
            if (message_type_A == 0 and message_type_B == 0 and self.bound == 0): # BIND REQUEST
                print("Bind request found from radio: " + str(sender))
                self.set_mode(MODE.TX)
                self.write_payload([self.fc_id, sender, 0, 1]) # Send BIND ACCEPT
                print("Accept sent")
            elif (message_type_A == 0 and message_type_B == 2 and self.bound == 0): # BIND ACCEPTANCE ACKNOWLEDGED
                print("Bind accept acknowledged")
                self.radio_id = sender
                self.bound = 1
                print("Now bound to radio" + str(sender))
                self.set_mode(MODE.TX)
                self.write_payload([self.fc_id, sender, 0, 4]) # Send REQUEST AXES
            elif (message_type_A == 0 and message_type_B == 3 and self.bound == 1): # AXIS CONTROL
                pitch = payload[4] * 256 + payload[5]
                roll = payload[6] * 256 + payload[7]
                throttle = payload[8] * 256 + payload[9]
                yaw = payload[10] * 256 + payload[11]
                aux1 = payload[12] * 256 + payload[13]
                aux2 = payload[14] * 256 + payload[15]
                aux3 = payload[16] * 256 + payload[17]
                aux4 = payload[18] * 256 + payload[19]
                msg = Int16MultiArray()
                msg.data = [throttle, roll, pitch, yaw, aux1, aux2, aux3, aux4]
                pub.publish(msg)
        
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

    

if __name__ == '__main__':

    if not radio.begin():
        raise RuntimeError("radio hardware is not responding")
    
    base_address = b"base_station"
    fc_address = b"flight_controller"

    radio.setPALevel(RF24_PA_LOW)

    radio.openReadingPipe(1, base_address)

    radio.payloadSize = 18

    rospy.init_node('radio_transceiver')
    global pub
    pub = rospy.Publisher('/rx_tx', Int16MultiArray, queue_size=10)
    rate = rospy.Rate(2)

    await_request()
    