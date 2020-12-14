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
#from SX127x.LoRaArgumentParser import LoRaArgumentParser
from SX127x.board_config import BOARD
import configparser

config = configparser.ConfigParser()
config.read('config.ini')

BOARD.setup()
BOARD.reset()
#parser = LoRaArgumentParser("Lora tester")


class mylora(LoRa):
    def __init__(self, verbose=False):
        super(mylora, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6)
        self.bound = 0
        self.fc_id = int(config['fc_radio_main']['fc_id'])
        self.radio_id = 0
        self.broadcast_id = 255

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
                pitch = payload[4]
                roll = payload[5]
                throttle = payload[6]
                yaw = payload[7]
                aux1 = payload[8]
                aux2 = payload[9]
                aux3 = payload[10]
                aux4 = payload[11]
                print("pitch: " + str(pitch) + "    roll: " + str(roll) + "    throttle: " + str(throttle) + "    yaw: " + str(yaw) + "    aux1: " + str(aux1) + "    aux2: " + str(aux2) + "    aux3: " + str(aux3) + "    aux4: " + str(aux4))
                self.set_mode(MODE.TX)
                self.write_payload([self.fc_id, sender, 0, 4]) # Send REQUEST AXES
        
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

    def on_tx_done(self):
        print("\nTxDone")
        print(self.get_irq_flags())

    def on_cad_done(self):
        print("\non_CadDone")
        print(self.get_irq_flags())

    def on_rx_timeout(self):
        print("\non_RxTimeout")
        print(self.get_irq_flags())

    def on_valid_header(self):
        print("\non_ValidHeader")
        print(self.get_irq_flags())

    def on_payload_crc_error(self):
        print("\non_PayloadCrcError")
        print(self.get_irq_flags())

    def on_fhss_change_channel(self):
        print("\non_FhssChangeChannel")
        print(self.get_irq_flags())

    def start(self):          
        while True:
            self.reset_ptr_rx()
            self.set_mode(MODE.RXCONT) # Receiver mode
            while True:
                pass

lora = mylora(verbose=False)
#args = parser.parse_args(lora) # configs in LoRaArgumentParser.py

#     Slow+long range  Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. 13 dBm
lora.set_pa_config(pa_select=1, max_power=21, output_power=15)
lora.set_bw(BW.BW500)
#lora.set_freq(433.0) 
lora.set_coding_rate(CODING_RATE.CR4_5)
lora.set_spreading_factor(7)
lora.set_rx_crc(True)
#lora.set_lna_gain(GAIN.G1)
#lora.set_implicit_header_mode(False)
#lora.set_low_data_rate_optim(True)

#  Medium Range  Defaults after init are 434.0MHz, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on 13 dBm
#lora.set_pa_config(pa_select=1)


assert(lora.get_agc_auto_on() == 1)

try:
    print("START")
    lora.start()
except KeyboardInterrupt:
    sys.stdout.flush()
    print("Exit")
    sys.stderr.write("KeyboardInterrupt\n")
finally:
    sys.stdout.flush()
    print("Exit")
    lora.set_mode(MODE.SLEEP)
    BOARD.teardown()