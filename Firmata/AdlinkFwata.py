#!/bin/python

import pyfirmata
import time
from time import sleep
from pyfirmata import Arduino, util
#import pyserial as serial


SYSEX_TMP102_REQUEST = 0x64
SYSEX_TMP102_REPLY = 0x65
SYSEX_LED_BLINK = 0x66
SYSEX_ACCEL_REQUEST = 0x67
SYSEX_ACCEL_REPLY = 0x68
SYSEX_COUNTER_REQUEST = 0x62
SYSEX_COUNTER_REPLY = 0x63
SYSEX_RTD_REQUEST = 0x80
SYSEX_RTD_REPLY = 0x81
SYSEX_RS485_REQUEST = 0x82
SYSEX_RS485_REPLY = 0x83
SYSEX_IOEXP_REQUEST = 0x84
SYSEX_IOEXP_REPLY = 0x85
SYSEX_CANBUS_REQUEST = 0x86
SYSEX_CANBUS_REPLY = 0x87
SYSEX_RS485_BAUDRATE_REQUEST = 0x88
SYSEX_RS485_BAUDRATE_REPLY = 0x89
SYSEX_MCUVersion_REQUEST = 0x8A
SYSEX_MCUVersion_REPLY = 0x8B
SYSEX_QSPI_REQUEST = 0x8C
SYSEX_QSPI_REPLY = 0x8D
SYSEX_SDCARD_REQUEST = 0x8E
SYSEX_SDCARD_REPLY = 0x8F
SYSEX_RESET_COUNTER_REQUEST = 0x90
SYSEX_RESET_COUNTER_REPLY = 0x91

SET_DIGITAL_PIN_VALUE = 0xF5

class GEABoard(pyfirmata.Arduino):
    counter = None
    accel = None
    temp = None
    RTD = None
    RTD1 = None
    RTD2 = None
    RS485 = None
    IOEXP  = None
    CANBUS = None
    RS485_BAUD = None
    MCU_Version = None
    QSPI = None
    count = None
    reset_counter = None
    sdcard = None
    sdcard_size = None
    
    def _handle_i2c(self, *data):
        print("I2C REPLY Data: received", {data})

    def _handle_tmp102(self, *data):
        self.temp = (data[0] + (data[2] << 7) + (data[4] << 14))/100 
        
    def _handle_analog(self, pin_nr, lsb, msb):
        print("handle analog" + str(pin_nr))

    def _handle_accel(self, *data):     
        #print("accel REPLY: received", data)
        if data[4] & 0x02:
            x = ((-1 & ~0xFFF) | (data[0] + (data[2] << 7) + (data[4] << 14)))
        else:
            x = (data[0] + (data[2] << 7) + (data[4] <<14))

        if data[10] & 0x02:
            y = ((-1 & ~0xFFF) | (data[6] + (data[8] << 7) + (data[10] << 14)))
        else:
            y = (data[6] + (data[8] << 7) + (data[10] <<14))

        if data[16] & 0x02:
            z = ((-1 & ~0xFFF) | (data[12] + (data[14] << 7) + (data[16] << 14)))
        else:
            z = (data[12] + (data[14] << 7) + (data[16] <<14))

        self.accel = [x, y, z]

    def _handle_counter(self, *data):     
        #print("counter REPLY: received", data)
        self.counter = data[0] + (data[2] << 7) + (data[4] << 14) + (data[6] << 21)
        #print("count: ", self.counter);
    
    def _handle_RTD(self, *data):
        #print("RTD REPLY: received", data)
        if data[6] & 0x40:
            self.RTD = ((-1 & ~0xFFFF) | (data[0] + (data[2] << 7) + (data[4] << 14) + (data[6] << 21)))/100    
        else:        
            self.RTD = (data[0] + (data[2] << 7) + (data[4] << 14) + (data[6] << 21))/100    
    def _handle_rs485(self, *data):
        #print(len(data))
        self.RS485 = []
        l = int(len(data)/2)
        #print(l)
        for i in range(0,l):
            #print(data[i*2])
            self.RS485.append(data[i*2]+ (data[i*2+1] << 7))
        #self.RS485 = value
        print("RS485 REPLY Data: ", self.RS485)
    def _handle_canbus(self, *data):
        #print(len(data))
        self.CANBUS = []
        l = int(len(data)/2)
        #print(l)
        for i in range(0,l):
            #print(data[i*2])
            self.CANBUS.append(data[i*2]+ (data[i*2+1] << 7))
        #self.CANBUS = value
        print("CANBUS REPLY Data: ", self.CANBUS)
    def _handle_ioexpander(self, *data):
        #print(len(data))
        #print(data)
        #self.IOEXP = data
        self.IOEXP = []
        self.IOEXP.append(data[0]+ (data[1] << 7))
    def _handle_rs485_baudrate(self, *data):
        self.RS485_BAUD = (data[6] << 21) + (data[4] << 14) + (data[2] << 7) + (data[0])   
        print("RS485 Baud Rate: ", self.RS485_BAUD)
    def _handle_mcuversion(self, *data):
        #print(data)
        self.MCU_Version = ""
        l = int(len(data)/2)
        #print(l)
        for i in range(0,l):
            #print(data[i*2])
            self.MCU_Version += str(chr(data[i*2]))
        #self.MCU_Version = data
        print("MCU REPLY Data: ", self.MCU_Version)
    def _handle_qspi(self, *data):
        #print(len(data))
        self.QSPI = []
        l = int(len(data)/2)
        #print(l)
        for i in range(0,l):
            #print(data[i*2])
            #print(data[i*2+1])
            self.QSPI.append(data[i*2]+ (data[i*2+1] << 7))
        #self.CANBUS = value
        #print("QSPI REPLY Data: ", self.QSPI)
    def _handle_reset_counter(self, *data):     
        #print("counter REPLY: received", data)
        self.reset_counter = data[0] + (data[2] << 7) + (data[4] << 14) + (data[6] << 21)
        #print("count: ", self.counter);
    def _handle_SDCard(self, *data):
        #print(len(data))
        self.sdcard = []
        l = int(len(data)/2)
        #print("data=")
        #print(data)
        for i in range(0,l):
            #print(data[i*2])
            #print(data[i*2+1])
            self.sdcard.append(data[i*2]+ (data[i*2+1] << 7))
        self.sdcard_size = 0
        if l == 4:
            self.sdcard_size = data[0] + (data[2] << 7) + (data[4] << 14) + (data[6] << 21)

        #self.CANBUS = value
        #print("QSPI REPLY Data: ", self.QSPI)



class AdlinkFwata:
#    def __init__(self, dev="/dev/ttyACM0"):
    def __init__(self, dev):  # For Windows Com port
        #self.board = pyfirmata.Arduino(dev)
        pyfirmata.pyfirmata.BOARD_SETUP_WAIT_TIME=0.1
        
        try: 
            # GEA pin assignment
            # self.geapins = {
            #         'din': [28, 23, 37, 36, 35, 34, 33, 32],
            #         'key': [31, 30],
            #         'dout': [45, 44, 41, 40, 43, 42, 2, 3],
            #         'relay': [48, 49, 46],
            #         'led': [12, 13, 88, 76, 75],
            #         'ain': [10, 11, 12, 3, 9, 7, 8],
            #         'aout': [0,1]
            # }
            self.geapins = {
                    'din': [2, 3, 4, 5, 6, 26, 27, 28],  # verify 1
                    'key': [22, 23],
                    'dout': [8, 9, 10, 11, 12, 29, 30, 31], # verify 1
                    'relay': [51, 52, 53], # verify 3
                    'led': [45, 46, 47],  # verify 1
                    'ain': [10, 11, 12, 3, 9, 7, 8], # hav value but not accurate
                    'aout': [0,1] # 1 is OK
            }
            self.board = GEABoard(dev)
            board_layout = {
                    #'digital': range(96),
                    #'analog': range(20), #[67, 68, 69, 70, 71, 72, 73, 74, 54, 55, 56, 57, 58, 59, 60, 61],
                    'digital': range(99),
                    'analog': range(16), #[67, 68, 69, 70, 71, 72, 73, 74, 54, 55, 56, 57, 58, 59, 60, 61],
                    'pwm': [0,1],
                    'disabled': []
            }
            self.board.setup_layout(board_layout)
            #self.it = pyfirmata.util.Iterator(self.board)
            #self.it.start()
            sleep(0.5)
            self.board.add_cmd_handler(pyfirmata.I2C_REPLY, self.board._handle_i2c)
            self.board.add_cmd_handler(SYSEX_TMP102_REPLY, self.board._handle_tmp102)
            self.board.add_cmd_handler(SYSEX_ACCEL_REPLY, self.board._handle_accel)
            self.board.add_cmd_handler(SYSEX_COUNTER_REPLY, self.board._handle_counter)
            self.board.add_cmd_handler(SYSEX_RTD_REPLY, self.board._handle_RTD)
            self.board.add_cmd_handler(SYSEX_RS485_REPLY, self.board._handle_rs485)
            self.board.add_cmd_handler(SYSEX_IOEXP_REPLY, self.board._handle_ioexpander)
            self.board.add_cmd_handler(SYSEX_CANBUS_REPLY, self.board._handle_canbus)
            self.board.add_cmd_handler(SYSEX_RS485_BAUDRATE_REPLY, self.board._handle_rs485_baudrate)
            self.board.add_cmd_handler(SYSEX_MCUVersion_REPLY, self.board._handle_mcuversion)
            self.board.add_cmd_handler(SYSEX_QSPI_REPLY, self.board._handle_qspi)
            self.board.add_cmd_handler(SYSEX_RESET_COUNTER_REPLY, self.board._handle_reset_counter)
            ##self.board.add_cmd_handler(ANALOG_MESSAGE, self.board._handle_analog)
            self.board.add_cmd_handler(SYSEX_SDCARD_REPLY, self.board._handle_SDCard)
        except Exception as e:
            print(e)

    def digitalWrite(self, pin, value):
        if pin >= 16:
            print("Digital Output : 0 - 7, Relay 8 - 10, LED 11 - 15");
            raise ValueError()

        if value > True:
            raise ValueError()
        try:
            if pin < 8:
                opin = self.geapins['dout'][pin]
            elif pin < 11:
                opin = self.geapins['relay'][pin-8]
            elif pin < 16:
                opin = self.geapins['led'][pin-11]
            else:
                print("Digital Output : 0 - 7, Relay 8 - 10, LED 11 - 15");
                return False
            #self.board.digital[opin].mode = pyfirmata.OUTPUT
            mybytes = [opin, value]
            self.board.send_sysex(SET_DIGITAL_PIN_VALUE, mybytes)
           # self.board.digital[opin].write(value)
        except Exception as e:
            print("Exception: ", e)
        return True

    def digitalRead(self, pin):
        if pin >= 10:
            print("Digital In : 0 - 7, Push Button Key 8 - 9");
            raise ValueError()

        if pin < 8:
            opin = self.geapins['din'][pin]
        elif pin < 11:
            opin = self.geapins['key'][pin-8]
        else:
            print("Digital In : 0 - 7, Push Button Key 8 - 9");
            return False
        self.board.digital[opin].mode = pyfirmata.INPUT
        self.board.digital[opin].enable_reporting()
        #self.it = pyfirmata.util.Iterator(self.board)
        #self.it.start()
        try:
            data = self.board.digital[opin].read()
            return data
        except Exception as e:
            return None

    def analogRead(self, pin):
        gain_c = 0.46684
        #gain_v = 0.24812
        gain_v = 0.19936 
        LSB = 0.00061
        if pin > 6:
            raise ValueError()
        
        try:
            apin = self.geapins['ain'][pin]
            try:
                #self.it = pyfirmata.util.Iterator(self.board)
                #self.it.start()
                anaIn = self.board.analog[apin]
                anaIn.type = pyfirmata.ANALOG;
                anaIn.enable_reporting()
                adc_data = anaIn.read()
                #print(adc_data)
                adc_data = (float(adc_data)*1023)
                if(apin == 10):
                    data = (adc_data*LSB)/gain_v
                else:    
                    data = (adc_data*LSB)/gain_c/249*1000
                return data
            except Exception as e:
                #print(e)
                return None
            return anaIn.read()
        except Exception as e:
            print(e)
            return None

    def analogWrite(self, pin, value):
        if pin > 5:
            raise ValueError()
        apin = self.geapins['aout'][pin]
        pinD = "d:" + str(apin) + ":p"
        try:
            anaOut = self.board.get_pin(pinD)
            anaOut.write(value)
        except Exception as e:
            print(e)

    def analogExtendedWrite(self, pin, value):
        if pin > 2:
            raise ValueError()
        apin = self.geapins['aout'][pin]
        try:
            #self.it = pyfirmata.util.Iterator(self.board)
            #self.it.start()
            data = []
            data.append(apin)
            data.append(value & 0xff)
            data.append(value >> 7 & 0xff)
            data.append(value >> 14 & 0xff)
            self.board.send_sysex(pyfirmata.EXTENDED_ANALOG,data)
        except Exception as e:
            print(e)

    def i2cConfig(self, delay):
        try:
            #self.it = pyfirmata.util.Iterator(self.board)
            #self.it.start()
            data = []
            data.append(delay & 0xff)
            data.append(delay >> 8 & 0xff)
            self.board.send_sysex(pyfirmata.I2C_CONFIG,data)
            print("I2C_CONFIG Send Successfully")
            sleep(1)
        except Exception as e:
            print(e)

    def i2cRegRead(self, addr, reg):
        try:
            #self.it = pyfirmata.util.Iterator(self.board)
            #self.it.start()
            data = [] #bytearray([addr, 0x48, reg, 0x01])
            data.append(addr)
            data.append(0x48)
            data.append(reg) # 7 Bit Slave, Read Operation
            data.append(0x00)
            data.append(0x01)
            data.append(0x00)
            self.board.send_sysex(pyfirmata.I2C_REQUEST,data)
            print("I2C_REQUEST Send Successfully: ", {data})
            sleep(1)
        except Exception as e:
            print(e)

    def i2cRegWrite(self, addr, reg, val):
        try:
            #self.it = pyfirmata.util.Iterator(self.board)
            #self.it.start()
            data = []
            data.append(addr)
            data.append(0x00) # 7 Bit Slave, Write Operation
            data.append(reg)
            data.append(0x00)
            data.append(val)
            data.append(0x00)
            self.board.send_sysex(pyfirmata.I2C_REQUEST,data)
            sleep(1)
        except Exception as e:
            print(e)

    def getCounter(self):
        try:
            #self.it = pyfirmata.util.Iterator(self.board)
            #self.it.start()
            data = []
            data.append(0x00)
            self.board.send_sysex(SYSEX_COUNTER_REQUEST,data)
            sleep(1)
            return self.board.counter
        except Exception as e:
            print(e)

    def getAccel(self):
        try:
            #self.it = pyfirmata.util.Iterator(self.board)
            #self.it.start()
            data = []
            data.append(0x00)
            self.board.send_sysex(SYSEX_ACCEL_REQUEST,data)
            sleep(1)
            return self.board.accel
        except Exception as e:
            print(e)

    def getRTD(self, rtd):
        try:
            #self.it = pyfirmata.util.Iterator(self.board)
            #self.it.start()
            rtd = int(rtd)
            data = []
            data.append(0x00)
            data.append(rtd)
            self.board.send_sysex(SYSEX_RTD_REQUEST,data)
            sleep(1)
            return self.board.RTD
        except Exception as e:
            print(e)

    def getTMP102Temp(self):
        try:
            #self.it = pyfirmata.util.Iterator(self.board)
            #self.it.start()
            data = []
            data.append(0x00) # 7 Bit Slave, Write Operation
            self.board.send_sysex(SYSEX_TMP102_REQUEST,data)
            sleep(1)
            return (self.board.temp)
        except Exception as e:
            print(e)

    def blinkLED(self, pin, delay):
        try:
            #self.it = pyfirmata.util.Iterator(self.board)
            #self.it.start()
            data = []
            dpin = self.geapins['led'][pin]
            data.append(dpin)
            data.append(delay & 0x7f)
            data.append((delay >> 7) & 0x7f)
            #print("blinkLED data: ", {data})
            self.board.send_sysex(SYSEX_LED_BLINK, data)
            sleep(1)
        except Exception as e:
            print(e)

    def writeRS485(self, value):
        try:
            #self.it = pyfirmata.util.Iterator(self.board)
            #self.it.start()
            data = []
            size = len(value)
            #print(size)
            #for i in range(0,size):
            #    if(int(value[i]) >= 240):  # 240 = 0xF0
            #        data.append(240)
            #        data.append(int(value[i])-240)
            #    else:
            #        data.append(int(value[i]))
            for i in range(0,size):
                if(int(value[i]) == 246 or int(value[i]) == 247):  # 240 = 0xF0
                    data.append(246)
                    data.append(int(value[i])-246)
                else:
                    data.append(int(value[i]))
            print("Send Data",data)
            self.board.send_sysex(SYSEX_RS485_REQUEST, data)
            sleep(1)
        except Exception as e:
            print(e)
    def writeRS485_baudrate(self, value):
        try:
            #self.it = pyfirmata.util.Iterator(self.board)
            #self.it.start()
            data = []
            size = len(value)
            #print(size)
            #for i in range(0,size):
            #    if(int(value[i]) >= 240):  # 240 = 0xF0
            #        data.append(240)
            #        data.append(int(value[i])-240)
            #    else:
            #        data.append(int(value[i]))
            for i in range(0,size):
                if(int(value[i]) == 246 or int(value[i]) == 247):  # 240 = 0xF0
                    data.append(246)
                    data.append(int(value[i])-246)
                else:
                    data.append(int(value[i]))
            print("Send Data",data)
            self.board.send_sysex(SYSEX_RS485_BAUDRATE_REQUEST, data)
            sleep(1)
        except Exception as e:
            print(e)
    def writeCAN(self, value):
        try:
            #self.it = pyfirmata.util.Iterator(self.board)
            #self.it.start()
            data = []
            size = len(value)
            #print(size)
            #for i in range(0,size):
            #    if(int(value[i]) >= 240):  # 240 = 0xF0
            #        data.append(240)
            #        data.append(int(value[i])-240)
            #    else:
            #        data.append(int(value[i]))
            for i in range(0,size):
                if(int(value[i]) == 246 or int(value[i]) == 247):  # 240 = 0xF0
                    data.append(246)
                    data.append(int(value[i])-246)
                else:
                    data.append(int(value[i]))
            if (size > 1 and data[1] > 7):
                print("Error: CAN Bus id should be less than 2048, first byte should be less than 7!")
            else:
                print("Send Data",data)
                self.board.send_sysex(SYSEX_CANBUS_REQUEST, data)
                sleep(1)
        except Exception as e:
            print(e)
    def readIOEXP(self, ioexp):
        try:
            #self.it = pyfirmata.util.Iterator(self.board)
            #self.it.start()
            #print("readIOEXP")
            ioexp = int(ioexp)
            data = []
            data.append(0x00)
            data.append(ioexp)
            self.board.send_sysex(SYSEX_IOEXP_REQUEST,data)
            sleep(1)
            return self.board.IOEXP
        except Exception as e:
            print(e)
    def getMCUversion(self, value):
        try:
            #self.it = pyfirmata.util.Iterator(self.board)
            #self.it.start()
            data = []
            self.board.send_sysex(SYSEX_MCUVersion_REQUEST, data)
            sleep(1)
            return self.board.MCU_Version
        except Exception as e:
            print(e)
    def writeqspi(self, value):
        try:
            #self.it = pyfirmata.util.Iterator(self.board)
            #self.it.start()
            data = []
            size = len(value)
            #print(size)

            #for i in range(0,size):
            #    if(int(value[i]) >= 240):  # 240 = 0xF0
            #        data.append(240)
            #        data.append(int(value[i])-240)
            #    else:
            #        data.append(int(value[i]))
            for i in range(0,size):
                if(int(value[i]) == 246 or int(value[i]) == 247):  # 240 = 0xF0
                    data.append(246)
                    data.append(int(value[i])-246)
                else:
                    data.append(int(value[i]))
            #for i in range(0,size):
            #    if(int(value[i]) >= 128):  # 240 = 0xF0
            #        data.append(int(value[i])-128)
            #        data.append(1)
            #    else:
            #        data.append(int(value[i]))
            #        data.append(0)
                    
            print("Send Data",data)
            self.board.send_sysex(SYSEX_QSPI_REQUEST, data)
            func_id = data[0]
            if func_id == 0:
                sleep(1)
                flash_id = (int(self.board.QSPI[3]) << 24) + (int(self.board.QSPI[2]) << 16) + (int(self.board.QSPI[1]) << 8) + int(self.board.QSPI[0])
                print("QSPI REPLY Data: id = ", hex(flash_id))
            if func_id == 1:
                print("Erase flash ....")
                index = data[1]
                if index == 0:
                    sleep(21)                
                    result = self.board.QSPI[0]
                    if result == 1:
                        print("QSPI REPLY Data: erase chip SUCCESS")
                    else:
                        print("QSPI REPLY Data: erase chip FAIL")
                        
                if index == 1:
                    sleep(2)
                    result = self.board.QSPI[0]
                    if result == 1:
                        no = (value[2] << 24) +  (value[3] << 16) + (value[4] << 8) +(value[5])
                        print("QSPI REPLY Data: erase sector",no,"SUCCESS")
                    else:
                        print("QSPI REPLY Data: erase sector",no,"FAIL")
                if index == 2:
                    sleep(2)                
                    result = self.board.QSPI[0]
                    if result == 1:
                        no = (data[2] << 24) +  (data[3] << 16) + (data[4] << 8) +(data[5])
                        print("QSPI REPLY Data: erase block",no,"SUCCESS")
                    else:
                        print("QSPI REPLY Data: erase block",no,"FAIL")
            if func_id == 2: # Wirte data
                print("Write flash...")
                sleep(2) 
                #print(self.board.QSPI[0])
                result = self.board.QSPI[0]
                if result > 0:
                    print("QSPI REPLY Data: SUCCESS", result, "bytes")
                else:
                    print("QSPI REPLY Data: Write data FAIL")
            if func_id == 3: #Read data
                print("Reading flash...")
                sleep(1) 
                print("QSPI REPLY Size:",len(self.board.QSPI))               
                print("QSPI REPLY Data:",self.board.QSPI)               
            #if func_id == 3:
        except Exception as e:
            print(e)

    def getresetCounter(self):
        try:
            #self.it = pyfirmata.util.Iterator(self.board)
            #self.it.start()
            data = []
            data.append(0x00)
            self.board.send_sysex(SYSEX_RESET_COUNTER_REQUEST,data)
            sleep(1)
            return self.board.reset_counter
        except Exception as e:
            print(e)

    def write_SD(self, value):
        try:
            data = []
            size = len(value)
            #print("size=")
            #print(size)
            #print(value)
            #print(value[0][0])
            try:
                size1 = len(value[0])
                if size1 != 1:
                    print("Error: parameter 1 not correct.")
                data.append(ord(value[0][0]))
            except Exception as e:
                print(e)
            try:
                size2 = len(value[1])
                #print("size2=",size2)
                for i in range(0,size2):
                    data.append(ord(value[1][i]))
            except Exception as e:
                print(e)
            func_code = value[0]
            #print("code=",func_code)

            # Special for write SD
            if func_code == "w":
                try:
                    w_id = value[1]
                    if w_id == "0":
                        size3 = len(value[2])
                        #print("size3=",size3)
                        for i in range(0,size3):
                            data.append(ord(value[2][i]))
                        #print(w_id)
                    elif w_id == "1":
                        size3 = len(value[2])
                        #print("size3=",size3)
                        for i in range(0,size3):
                            data.append(ord(value[2][i]))
                        data.append(ord('\n'))
                        #print(w_id)
                        #print("SD : Write data 1:",data)  
                    elif w_id == "2":
                        #data1 = []
                        size = len(value)
                        #print("w_id =1")
                        #print(size)
                        for i in range(2,size):
                            if(int(value[i]) == 246 or int(value[i]) == 247):  # 240 = 0xF0
                                data.append(246)
                                data.append(int(value[i])-246)
                            else:
                                data.append(int(value[i]))     
                        #print(w_id)
                        print("SD : Write data :",data)  
                    else:
                        print("Command Parameter Error!")
                        return
                except Exception as e:
                    print(e)
            elif func_code == "r" or func_code == "l" :
                try:
                    #print(value[1])
                    w_id = value[1]
                    if w_id == "0" or w_id == "1":
                        size3 = len(value[1])
                        #size3 = len(value[2])
                        ##print("size3=",size3)
                        for i in range(0,size3):
                            data.append(ord(value[1][i]))
                        ##print(w_id)
                    else:
                        print("Command Parameter Error!")
                        return
                    #print(data)
                except Exception as e:
                    print(e)
            #print(func_id)

            self.board.send_sysex(SYSEX_SDCARD_REQUEST, data)
            func_id = data[0]
            #print(func_id)
            if func_code == "r":
                sleep(1) 
                #print("SD Read Data:")
                size = len(self.board.sdcard)
                text = ''
                for i in range(0,size):
                    text = text + chr(self.board.sdcard[i])
                print(text)         
                #print(self.board.sdcard)
            elif func_code == "w":
                #print(" a File: ",value[1])
                sleep(1) 
                result = self.board.sdcard[0]
                if result == 0:
                    print("Result: Failed")
                else:
                    print("Result: OK")

            elif func_code == "l":
                sleep(1) 
                size = len(self.board.sdcard)
                text = ''
                for i in range(1,size):
                    text = text + chr(self.board.sdcard[i-1])
                print(text)         
            elif func_code == "d":
                sleep(1)
                result = self.board.sdcard[0]
                if result == 0:
                    print("Result: Failed")
                else:
                    print("Result: OK")
            elif func_code == "k":
                sleep(1)
                result = self.board.sdcard[0]
                if result == 0:
                    print("Result: Failed")
                else:
                    print("Result: OK")
            elif func_code == "o":
                sleep(1)
                result = self.board.sdcard[0]
                if result == 0:
                    print("Result: Failed")
                else:
                    print("Result: OK")
            elif func_code == "c":
                sleep(1)
                result = self.board.sdcard[0]
                if result == 0:
                    print("Result: Failed")
                else:
                    print("Result: OK")
            elif func_code == "m":
                sleep(1)
                result = self.board.sdcard[0]
                if result == 0:
                    print("Result: Failed")
                else:
                    print("Result: OK")
            elif func_code == "i":
                sleep(1)
                #print("SD Read Data: ", )
                #result = self.board.sdcard[0]
                print("Result : Size =", self.board.sdcard_size, "MB")
            
            else:
                print("Command Parameter Error!")

        except Exception as e:
            print(e)            

    def start_firmata(self):
        self.it = pyfirmata.util.Iterator(self.board)
        self.it.start()

