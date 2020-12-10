import time
import Adafruit_ADS1x15
from SX127x.LoRa import *
from SX127x.board_config import BOARD

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

    def on_rx_done(self):
        BOARD.led_on()
        #print("\nRxDone")
        self.clear_irq_flags(RxDone=1)
        payload = self.read_payload(nocheck=True )# Receive INF
        print ("Receive: ")
        mens=bytes(payload).decode("utf-8",'ignore')
        mens=mens[2:-1] #to discard \x00\x00 and \x00 at the end
        print(mens)
        BOARD.led_off()
        if mens=="INF":
            print("Received data request INF")
            time.sleep(2)
            for i in range(4):
                stick_readings[i] = round((adc.read_adc(i, gain=GAIN) / 30700) * 255)
            print (stick_readings)
            self.write_payload([255, 255, 0, 0, 68, 65,
            , 0]) # Send DATA RASPBERRY PI
            self.set_mode(MODE.TX)
        time.sleep(2)
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
            


##



