# OT2IT

1. Install the bootloader 
	https://github.com/ADLINK/uf2-samdx1/

2. Setup Arduino environment with ADLINK SAMD Boards Installed 
	https://raw.githubusercontent.com/ADLINK/OT2IT/main/packages_adlink_ot2it.json
	
3. Copy the SAME54 CMSIS Files into arduino core:
	`/arduino/tools/CMSIS-Atmel/1.2.0/CMSIS/Device/ATMEL`
	Copy the CMSIS files into `<ARDUINO DIR>\packages\arduino\tools\CMSIS-Atmel\1.2.0\CMSIS\Device\ATMEL`
	For Ex: `<ARDUINO DIR>` = `%LOCALAPPDATA%\Arduino15\` on Windows
	
4. Install the OT2ITEthernet Library (`OT2ITEth.zip`)
	Using Sketch->Include Library -> Add .Zip Library
	
5. Open the Ethernet Sample Sketch in Arduino
	`OT2ITEth/examples/ethernet_rec/ethernet_rec.ino`
	
6. Select the OT2IT board & Port
	Board Selection : Tools -> Boards -> Adlink SAMD Boards -> Adlink OT2IT
	Port Selection : Tools -> Ports -> COM with Adlink OT2IT
	
7. Compile the Sketch
	Using Sketch -> Verify / Compile
	
8. Upload the sketch into the board
	Double click the reset button of OT2IT board to make the board to enter into upload mode
	Then Upload the sketch : Sketch -> Upload
	
9. Open the serial monitor
	Tools -> Serial Monitor
	In monitor, OT2IT assigned with IP Address will be displayed.
	To Test -> ping the IP address from Host machine
	To Test web -> enter `http://<OT2IT IP Address>` on the web browser