import time
import Adafruit_ADS1x15
from SX127x.LoRa import *
from SX127x.board_config import BOARD
import configparser

config = configparser.ConfigParser()
config.read('config.ini')

#global radio_id
#global broadcast_id
#global fc_id
#fc_id = 255
#radio_id = config['radio_main']['radio_id']
#broadcast_id = 255

## ADC SETUP
adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1
stick_readings = [0]*4

## LORA SETUP

BOARD.setup()
BOARD.reset()
#parser = LoRaArgumentParser("Lora tester")


class mylora(LoRa):
    def __init__(self, verbose=False):
        super(mylora, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6)
        self.bound=0
        self.radio_id = int(config['radio_main']['radio_id'])
        self.fc_id = 255
        self.broadcast_id = 255

    def on_rx_done(self):
        BOARD.led_on()
        #print("\nRxDone")
        self.clear_irq_flags(RxDone=1)
        payload = self.read_payload(nocheck=True )# Receive INF
        print ("Receive: ")
        print(payload)
        sender = payload[0]
        recipient = payload[1]
        message_type_A = payload[2]
        message_type_B = payload[3]
        BOARD.led_off()
        if (recipient == self.radio_id):
            if (message_type_A == 0 and message_type_B == 1 and self.bound == 0): # BIND ACCEPTED
                print("Bind accepted")
                print("FC ID: " + str(sender))
                self.fc_id = sender
                self.bound = 1
                self.set_mode(MODE.TX)
                self.write_payload([self.radio_id, self.fc_id, 0, 2]) # Send BIND ACK
            elif (message_type_A == 0 and message_type_B == 4 and self.bound == 1): # AXES REQUESTED
                for i in range(4):
                    stick_readings[i] = round((adc.read_adc(i, gain=GAIN) / 30700) * 255)
                print (stick_readings)
                pitch = stick_readings[0]
                roll = stick_readings[1]
                throttle = stick_readings[2]
                yaw = stick_readings[3]
                aux1 = 0
                aux2 = 0
                aux3 = 0
                aux4 = 0
                self.set_mode(MODE.TX)
                self.write_payload([self.radio_id, self.fc_id, 0, 3, pitch, roll, throttle, yaw, aux1, aux2, aux3, aux4]) # Send axis commands
            
            #time.sleep(2)
            #for i in range(4):
            #    stick_readings[i] = round((adc.read_adc(i, gain=GAIN) / 30700) * 255)
            #print (stick_readings)
        #time.sleep(1)
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)
        #time.sleep(0.5)

    def on_tx_done(self):
        
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
        self.bound = 0
        while True:
            while (self.bound==0):
                print ("Sending bind request")
                self.set_mode(MODE.TX)
                self.write_payload([self.radio_id, self.broadcast_id, 0, 0]) # Broadcast BIND REQUEST
                #time.sleep(3) # there must be a better solution but sleep() works
                self.reset_ptr_rx()
                self.set_mode(MODE.RXCONT)
            
                start_time = time.time()
                while (time.time() - start_time < 15): # wait until receive data or 10s
                    pass
            
            

            while (self.bound == 1):
                for i in range(4):
                    stick_readings[i] = round((adc.read_adc(i, gain=GAIN) / 30700) * 255)
                print (stick_readings)
                pitch = stick_readings[0]
                roll = stick_readings[1]
                throttle = stick_readings[2]
                yaw = stick_readings[3]
                aux1 = 0
                aux2 = 0
                aux3 = 0
                aux4 = 0
                self.set_mode(MODE.TX)
                self.write_payload([self.radio_id, self.fc_id, 0, 3, pitch, roll, throttle, yaw, aux1, aux2, aux3, aux4]) # Send axis commands
                self.reset_ptr_rx()
                self.set_mode(MODE.RXCONT)
            
                start_time = time.time()
                while (time.time() - start_time < 4): # wait until receive data or 10s
                    pass

            self.bound=0
            self.reset_ptr_rx()
            self.set_mode(MODE.RXCONT) # Receiver mode
            time.sleep(10)

lora = mylora(verbose=False)
#args = parser.parse_args(lora) # configs in LoRaArgumentParser.py

#     Slow+long range  Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. 13 dBm
lora.set_pa_config(pa_select=1, max_power=21, output_power=15)
#lora.set_freq(433.0) 
lora.set_bw(BW.BW500)
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