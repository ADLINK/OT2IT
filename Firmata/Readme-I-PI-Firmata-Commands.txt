Setup: PX30 booted with Debian Image

To Run the Test Cases:

1. To Read from Digital Input ( Note: pin number => 0 - 7 for Digital Input, 8 - 9 for Key1 & Key2 )
    $ sudo python FwataTest.py -i <pin number>

2. To Write on Digital Output ( Note: Pin number => 0 - 7 for Digital Output, 8 - 10 for Relay 0 - 2, 11 for MCU_LED_G, 12 for MCU_LED_R, 13 for Blue LED
    $ sudo python FwataTest.py -o <pin number>,<0|1>
	For Example: To write 1 on DO0_MCUIO
	$ sudo python FwataTest.py -o 0,1
	For Example: To write 1 on RO0_MCUIO	
	$ sudo python FwataTest.py -o 8,1

3. To Read from Analog Input ( Voltage )
    $ sudo python FwataTest.py -a <pin number>
	For Example: To read from analog pin AII_A0_in
	$ sudo python FwataTest.py -a 0

4. To Read from Analog Input ( Current )
    $ sudo python FwataTest.py -a <pin number>
	For Example: To read from analog current 0. Analog Current Input 0 => 1 and 5 => 6
	0 => AII_A0, 1 => AII_A1, 2 => AII_P0, 3 => AII_P1, 4 => AII_P2, 5 => AII_P3 
	$ sudo python FwataTest.py -a 1

5. To write into analog output    
    $ sudo python FwataTest.py -b <pin number>,<value from 0 to 4095>
	For Example: To write over analog pin AOV0. 0 for AOV0, 1 for AOV1
	$ sudo python FwataTest.py -b 0,4095 => 10.3V
	$ sudo python FwataTest.py -b 0,0 => 0V

6. To Read Pulse Count Value
    $ sudo python FwataTest.py -p

7. To Read Temperature value
    $ sudo python FwataTest.py -t

8. To Read Temperature value
    $ sudo python FwataTest.py -x
	
9. To Temperature from RTD
    $ sudo python FwataTest.py -d <RTD number>
	For Example: To read from temperature from RTD 0.
	$ sudo python FwataTest.py -d 0
	
10. To send and receive RS485 data
    $ sudo python FwataTest.py -s <func>,<data0,data1,data2,....>
	For Example: func =1, To send data 48,49,50 to RS485 and this will also get data from RS485
	$ sudo python FwataTest.py -s 1,48,49,50
	For Example: func = 0, Just to get data from rs485
	$ sudo python FwataTest.py -s 0

11. To set or get RS485 baud rate
    $ sudo python FwataTest.py -u <func>,<index>
	For Example: To get RS485 baud rate
	$ sudo python FwataTest.py -u 0
	For Example: To set RS485 baud rate
	//
	// index = 1 :   9600
	// index = 2 :  19200
	// index = 3 :  38400
	// index = 4 :  57600
	// index = 5 : 115200
	// index = other : not changed
	//
	$ sudo python FwataTest.py -u 1,index 

12. To read io expander io
    $ sudo python FwataTest.py -y <port number>
	For Example: To read io 0~7
	$ sudo python FwataTest.py -y 0
	For Example: To read io 8~15
	$ sudo python FwataTest.py -y 1

13. To send and receive CANBus data
    $ sudo python FwataTest.py -n <func>,<id0,id1>,<data0,data1,data2,....,data63>
	  where func = 1 --> send and recevie can bus id/data
	        func = 0 --> receive can bus id/data
			bus id (11 bits) = 256*id0 (3 bits) + id1 (8 bits),
			       id0 = 0~7 , id1 = 0~255
			data = <data0,data1,data2,....,data63> 
			       data size = 0~64
				   data size can be 0,1,2,3,4,5,6,7,8,12,16,20,24,,32,48,64
	For Example: func = 1, To send data 48,49,50 to CANBUS id = 2 and this will also get data from CANBus
	$ sudo python FwataTest.py -n 1,0,2,48,49,50
	For Example: func = 0, Just to get data from CANBUS
	$ sudo python FwataTest.py -n 0

14. To get firmata command version
    $ sudo python FwataTest.py -v 0
	For Example: To get firmata command version
	$ sudo python FwataTest.py -v 0

15. To get MCU FW version
    $ sudo python FwataTest.py -m 0
	For Example: To get MCU FW version
	$ sudo python FwataTest.py -m 0
	
16. To get Flash via qspi
    (1) Get flash id 
    $ sudo python FwataTest.py -q 0
	
    (2) Chip Erase flash 
    $ sudo python FwataTest.py -q 1,0
	
    (3) Sector Erase flash (sector number = 0 --> 2047)
    $ sudo python FwataTest.py -q 1,1,<sector number>
	For Example: To erase sector number 5
    $ sudo python FwataTest.py -q 1,1,5
	
    (4) Block Erase flash (block number = 0 --> 127)
    $ sudo python FwataTest.py -q 1,2,<block number>
	For Example: To erase block number 7
    $ sudo python FwataTest.py -q 1,2,7
	
    (5) Write flash
    $ sudo python FwataTest.py -q 2,<addr>,<data0,data1,data2....>
	For Example: To write data 0,1,2,3,4,5,6 at addr 256
    $ sudo python FwataTest.py -q 2,256,0,1,2,3,4,5,6
	
    (6) Read flash
    $ sudo python FwataTest.py -q 3,<addr>,<byte number>
	For Example: To read 10 bytes data at addr 256
    $ sudo python FwataTest.py -q 3,256,10

17. SD Card function, the card should be formatted as FAT32
    (1) Create a file
    $ sudo python FwataTest.py -f c,<filename>
      For example: To create a file test.txt
      $ sudo python FwataTest.py -f c,"test.txt"
      For example: To create a file /dir0/test.txt
      $ sudo python FwataTest.py -f c,"/dir0/test.txt"
    (2) Open a file or directory
    $ sudo python FwataTest.py -f o,<filename>
      For example: To open a file test.txt
      $ sudo python FwataTest.py -f o,"log/test.txt"
      For example: To open a file test.txt under directory log
      $ sudo python FwataTest.py -f o,"log/test.txt"

    (3) Create a directory
    $ sudo python FwataTest.py -f m,<directory>
      For example: To create a directory dir0
      $ sudo python FwataTest.py -f m,"dir0"

    (4) Write data to a opened file
    $ sudo python FwataTest.py -f w,<type>,<data>
	  type = 0 : write a string
	  type = 1 : write charactor
	  note: data length should be less 252
      For example: To write data to the opened file
	  [1] Open a existing file "test.txt"
      $ sudo python FwataTest.py -f o,"test.txt"
	  [2] then, write "112233445" to the opened file.
      $ sudo python FwataTest.py -f w,0,"1122334455"
	  [3] then, write "6677889900" with to the opened file.
      $ sudo python FwataTest.py -f w,0,"6677889900"
	  [4] then, write a return character with to the opened file.
      $ sudo python FwataTest.py -f w,1,10,13
	  

    (5) Read data from a file
    $ sudo python FwataTest.py -f r,<type>
		type = 0, read from the begining
		type = 1, continue to read
        For example: To read data 255 bytes from the opened file.
      $ sudo python FwataTest.py -f r,0
	    Contunue to read
      $ sudo python FwataTest.py -f r,1

    (6) List the files	
    $ sudo python FwataTest.py -f l,<type>
		type = 0, read from the begining
		type = 1, continue to read
        For example: To list the files from
      $ sudo python FwataTest.py -f l,0
      $ sudo python FwataTest.py -f l,1
	  
    (7) Delete a file
    $ sudo python FwataTest.py -f d,<filename>
        For example: To remove the file "test.txt"
      $ sudo python FwataTest.py -f d,"test.txt"
	  
    (8) Remove a empty directory
    $ sudo python FwataTest.py -f k,<directory>
      For example: To remove the directory "dir0"
      $ sudo python FwataTest.py -f k,"dir0"

    (9) Show SD card size (could be the size after format)
    $ sudo python FwataTest.py -f i,0
      For example: Show SD card size
      $ sudo python FwataTest.py -f i,0

18. Get Reset Counter
   When the device has been reset (power on), the counter will be 1, and then increase 1 each time command request
   $ sudo python FwataTest.py -g


