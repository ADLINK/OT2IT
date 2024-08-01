#!/usr/bin/env python3
from time import sleep
import sys
import getopt

from AdlinkFwata import AdlinkFwata

with open('device_port.txt', 'r') as device_port_file:
    device_port = device_port_file.read().strip()
#fwata = AdlinkFwata("/dev/ttyACM1")
fwata = AdlinkFwata(device_port)

firmata_version = "Firmata command version: 1.06"

#while True:
#    fwata.digitalWrite(6, 0)
#    fwata.digitalWrite(7, 1)
#    fwata.digitalWrite(8, 1)
#    sleep(1)
#    fwata.digitalWrite(7, 0)
#    sleep(1)
#    fwata.digitalWrite(8, 0)
#    sleep(1)
#    fwata.digitalWrite(6, 1)
#    sleep(1)
#    fwata.digitalWrite(7, 1)
#    sleep(1)
#    fwata.digitalWrite(8, 1)
#    sleep(1)

def printHelp():
    print("FwataTest.py Usage:")
    print("FwataTest.py --din=<digital pin>")
    print("FwataTest.py --dout=<digital pin>,<0|1>")
    print("FwataTest.py --anain=<analong pin>")
    print("FwataTest.py --anaout=<analog pin>,<value>")
    return

def main(argv):
    try:
        #opts, args = getopt.getopt(argv, "hi:o:a:b:d:r:w:c:tpx",["din=","dout=","anain=","anaout=","","i2cread=","i2cwrite=","blink=","getTMP","getCount","getAccel","rs485write"])
        opts, args = getopt.getopt(argv, "hi:u:o:a:b:d:r:w:c:s:y:n:m:q:tpx:e:v:f:g",["din=","rs485baud","dout=","anain=","anaout=","getRTD","i2cread=","i2cwrite=","blink=","version","mversion","getTMP","getCount","getAccel","rs485write","rs485read","ioexpread","canwrite","qspi","file","powergoodcount"])
    except getopt.GetOptError:
        printHelp()
        sys.exit(2)
    fwata.start_firmata()
    for opt,arg in opts:
        if opt in ("-i", "--din"):
            #print('--din='+str(arg))
            if int(arg) > 9:
                print("Error: Pin number is out of range, should be less than 10")
                #print(e)
                sys.exit()

            try:
                data = fwata.digitalRead(int(arg))
                if data is None:
                    sleep(1)
                    data = fwata.digitalRead(int(arg))
                if data is True:
                    result = 0
                else:
                    result = 1
                if result == 1: 
                    print("DigitalRead on pin"+str(arg) +"= " + str(data) + " ,(High)")
                else:
                    print("DigitalRead on pin"+str(arg) +"= " + str(data) + "  ,(Low)")
                #print("DigitalRead on pin"+str(arg) +"= " + str(data))
                #print("DigitalRead on pin"+str(arg) +"= " + str(result))
            except Exception as e:
                print('fwata exp')
                print(e)
                sys.exit()
        elif opt in ("-o", "--dout"):
            #print("--dout=", {arg})
            try:
                x=arg.split(',')
                if int(x[0]) > 15:
                    print("Error: Pin number is out of range, should be less than 16")
                    sys.exit()
                if int(x[1]) > 1:
                    print("Error: Value is out of range, should be 0 or 1")
                    sys.exit()
                fwata.digitalWrite(int(x[0]), int(x[1]))
            except Exception as e:
                print(e)
                sys.exit()
        elif opt in ("-a", "--anain"):
            try:
                if int(arg) > 9:
                    print("Error: Pin number is out of range, should be less than 7")
                    sys.exit()
                data = fwata.analogRead(int(arg))
                if data is None:
                    sleep(1)
                    data = fwata.analogRead(int(arg))
                    data = round(data, 2)
                    #sleep(1)
                if data is not None:
                    data = round(data, 2)
                    if(int(arg) == 0):    
                        print("Voltage on pin", str(arg), "=", str(data),"V")
                    else:    
                        print("Current readings on pin", str(arg), "=", str(data),"mA")
            except Exception as e:
                print(e)
                sys.exit()
        elif opt in ("-b", "--anaout"):
            #print("--anaout=", {arg})
            try:
                x=arg.split(',')
                if int(x[0]) > 1:
                    print("Error: Pin number is out of range, should be 0 or 1")
                    sys.exit()
                if int(x[1]) > 4095:
                    print("Error: Value is out of range, should less than 4096")
                    sys.exit()
                #fwata.analogWrite(int(x[0]), float(x[1]))
                fwata.analogExtendedWrite(int(x[0]), (int(x[1])))
            except Exception as e:
                print(e)
                sys.exit()
            print(x)

        elif opt in ("-r", "--i2cread"):
            #print("--i2cread=",{arg})
            try:
                x=arg.split(',')
                fwata.i2cConfig(1)
                fwata.i2cRegRead(int(x[0], 16), int(x[1], 16))
            except Exception as e:
                print(e)
                sys.exit()
        elif opt in ("-w", "--i2cwrite"):
            #print("--i2cwrite=",{arg})
            try:
                x=arg.split(',')
                fwata.i2cRegWrite(int(x[0], 16), int(x[1], 16), int(x[2], 16))
            except Exception as e:
                print(e)
                sys.exit()
        elif opt in ("-c", "--blink"):
            #print("--blink=",{arg})
            try:
                x=arg.split(',')
                fwata.blinkLED(int(x[0]), int(x[1]))
            except Exception as e:
                print(e)
                sys.exit()
        elif opt in ("-t", "--getTMP"):
            #print("--getTMP=",{arg})
            try:
                data = fwata.getTMP102Temp()
                print("TMP102 Temp: {} c".format(str(data)))
            except Exception as e:
                print(e)
        elif opt in ("-x", "--getAccel"):
            #print("--getAccel=",{arg})
            try:
                data = fwata.getAccel()
                print("Accelerometer[x,y,z]: ", str(data))
            except Exception as e:
                print(e)
        elif opt in ("-p", "--getCount"):
            #print("--getCount=",{arg})
            try:
                data = fwata.getCounter()
                #print("Pulse Count: ", data)
                #print("Freq: ", data, "Hz")
                print("Pulse Counts (in 100ms):", data)
            except Exception as e:
                print(e)
        elif opt in ("-d", "--getRTD"):
            #print("--getRTD=",{arg})
            if int(arg) > 1:
                print("Error: RTD number is out of range, should be less than 2")
                #print(e)
                sys.exit()
            try:
                data = fwata.getRTD(arg)
                print("Temp in RTD{}: {} c".format(str(arg),str(data)))
            except Exception as e:
                print(e)
        elif opt in ("-s", "--rs485write"):
            #print("--rs485write= ",{arg})
            try:
                #print(arg)
                x=arg.split(',')
                if int(x[0]) > 2:
                    print("Error: Function should be more than 2")
                    sys.exit()
                fwata.writeRS485(x)
                #print("execute")
            except Exception as e:
                print(e)
                sys.exit()
        elif opt in ("-u", "--rs485baud"):
            #print("--rs485baud= ",{arg})
            try:
                #print(arg)
                x=arg.split(',')

                if int(x[0]) > 2:
                    print("Error: Function should be more than 2")
                    sys.exit()
                if int(x[0]) == 1 and int(x[1]) > 5:
                    print("Error: Index is out of range, should less than 6")
                    sys.exit()

                fwata.writeRS485_baudrate(x)
                #print("execute")
            except Exception as e:
                print(e)
                sys.exit()
        elif opt in ("-y", "--ioexpread"):
            #print("--ioexpread= ",{arg})
            try:
                #print(arg)
                data = fwata.readIOEXP(arg)
                #print(arg)
                if (int(arg) > 1):
                    print("Error: Index is out of range, should less than 2")
                    sys.exit()

                if (int(arg) == 0):
                    print("IO Expander IO pins 7~0: {}".format(str(hex(data[0]))))
                else:
                    print("IO Expander IO pins 15~8: {}".format(str(hex(data[0]))))
                    
            except Exception as e:
                print(e)
        
        elif opt in ("-n", "--canwrite"):
            #print("--canwrite= ",{arg})
            try:
                #print(arg)
                x=arg.split(',')
                fwata.writeCAN(x)
                #print("execute")
            except Exception as e:
                print(e)
                sys.exit()
        elif opt in ("-m", "--mversion"):
            #print("--mcuversion= ",{arg})
            try:
                #print(arg)
                #x=arg.split(',')
                fwata.getMCUversion(arg[0])
                #print("execute")
            except Exception as e:
                print(e)
                sys.exit()
        elif opt in ("-q", "--qspi"):
            #print("--qspi= ",{arg})
            try:
                #print(arg)
                x=arg.split(',')
                #print(x)
                # Erase Function
                if x[0] == '1':
                    #print(len(x))
                    if len(x) < 2:
                        #print(len(x))
                        print("Error: Erase function parameter error!")
                        sys.exit()
                    if x[1] == '1' or x[1] == '2':
                        if len(x) < 3:
                            #print(len(x))
                            print("Error: Erase function parameter error!")
                            sys.exit()

                if x[0] == '1' and x[1] > '0' :
                    if x[1] > '2':
                        print("Error: Erase function index error!")
                        sys.exit()
                    if x[1] == '1':
                        y = int(x[2])

                        if y < 2048 :
                            del x[2:100]
                            x.append( (y >> 24) & 0xFF)
                            x.append( (y >> 16) & 0xFF)
                            x.append( (y >> 8)  & 0xFF)
                            x.append( (y & 0xFF ))
                        else:
                            print("Error: Sector number should be less than 2048")
                            sys.exit()
                    if x[1] == '2':
                        y = int(x[2])
                        if y < 128 :
                            del x[2:100]
                            x.append( (y >> 24) & 0xFF)
                            x.append( (y >> 16) & 0xFF)
                            x.append( (y >> 8)  & 0xFF)
                            x.append( (y & 0xFF ))
                        else:
                            print("Error: Bloc number should be less than 128")
                            sys.exit()
                #print(x) 
                # -------------------------------------------------------------
                if  x[0] == '2': # write
                    y = int(x[1])
                    if y < 8388608 :
                        del x[1]
                        x.insert(1, (y >> 24) & 0xFF)
                        x.insert(2, (y >> 16) & 0xFF)
                        x.insert(3, (y >> 8)  & 0xFF)
                        x.insert(4, (y & 0xFF ))
                        del x[252:1000]
                    else:
                        print("Error: Address should be less than 8388608")
                        sys.exit()
                if  x[0] == '3': # read
                    y = int(x[1])
                    z = int(x[2])
                    if y < 8388608 :
                        del x[1:100]
                        x.append( (y >> 24) & 0xFF)
                        x.append( (y >> 16) & 0xFF)
                        x.append( (y >> 8)  & 0xFF)
                        x.append( (y & 0xFF ))
                    else:
                        print("Error: Address should be less than 8388608")
                        sys.exit()
                    if z < 256 :
                        x.append( (z >> 8)  & 0xFF)
                        x.append( (z & 0xFF ))
                    else:
                        print("Error: Data number should be less than 256")
                        sys.exit()
                fwata.writeqspi(x)
                #print("execute")
            except Exception as e:
                print(e)
                sys.exit()
        elif opt in ("-v", "--version"):
            #print("--qspi= ",{arg})
            try:
                print(firmata_version)

            except Exception as e:
                print(e)
                sys.exit()
        # SD Card
        elif opt in ("-f", "--file"):
            #print("--file= ",{arg})
            try:
                #print(arg)
                x=arg.split(',')
                l = len(x)
                #print(l)
                if l <= 1:
                    print("Parameter not enough!")
                    print(e)
                    sys.exit()
                # print(x[0])
                # Erase Function
                if x[0] == 'c': # create a file
                    print("SD : Create a file",x[1])
                    #print(x[1])
                elif x[0] == 'o': # open a file
                    print("SD : Open a file or directory:",x[1])
                    #print(x[1])
                elif x[0] == 'm': # make a directory
                    print("SD : Make a directory:",x[1])
                elif x[0] == 'w': # write data to a file
                    if x[1] == '0':
                        print("SD : Write data :",x[2])
                    if x[1] == '1':
                        print("SD : Write data :",x[2],"\\n")
                    #print(x[1])
                elif x[0] == 'r': # read data from a file
                    if x[1] == '0':
                        print("SD : Read Data:")
                    if x[1] == '1':
                        print("SD : Continue to read data:")
                    #print(x[1])
                elif x[0] == 'l': # list files
                    if x[1] == '0':
                        print("SD : List directory files ")
                    if x[1] == '1':
                        print("SD : Continue to list directory files ")
                elif x[0] == 'd': # delete a file
                    print("SD : Delete a file: ",x[1])
                    #print(x[1])
                elif x[0] == 'k': # delete a directory
                    print("SD : Delete a directory: ",x[1])
                    #print(x[1])
                elif x[0] == 'i':
                    print("SD : Get Card Size")
                fwata.write_SD(x)

            except Exception as e:
                print(e)
                sys.exit()
        elif opt in ("-g", "--goodcountpower"):
            #print("--power on or reset= ",{arg})

            try:
                data = fwata.getresetCounter()
                #print("Pulse Count: ", data)
                #print("Freq: ", data, "Hz")
                #data = 100
                print("Reset Counter : ", data)

            except Exception as e:
                print(e)
                sys.exit()

if __name__ == "__main__":
    main(sys.argv[1:])

