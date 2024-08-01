#include <Ethernet.h>

#include <Adafruit_QSPI.h>
#include <Adafruit_QSPI_Flash.h>

/*485485
  Firmata is a generic protocol for communicating with microcontrollers
  from software on a host computer. It is intended to work with
  any host computer software package.

  To download a host software package, please click on the following link
  to open the list of Firmata client libraries in your default browser.

  https://github.com/firmata/arduino#firmata-client-libraries

  Copyright (C) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
  Copyright (C) 2010-2011 Paul Stoffregen.  All rights reserved.
  Copyright (C) 2009 Shigeru Kobayashi.  All rights reserved.
  Copyright (C) 2009-2016 Jeff Hoefs.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.

  Last updated August 17th, 2017
*/
#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function
#include <Servo.h>
#include <Wire.h>
#include <Firmata.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define DEV_I2C Wire
#define GEA_BOARD 1
//#include <LIS2DW12.h>
#include "SPI.h"
#include <Adafruit_MAX31865.h>
/* TMP102 Sensor Library */
#include <SparkFunTMP102.h>
TMP102 sensor0;
/* LIS2DW Accelerometer Library */
#include <LIS2DW12Sensor.h>
#include <lis2dw12_reg.h>

/* ACCANFD Library */
#define CAN0_MESSAGE_RAM_SIZE (0)
#define CAN1_MESSAGE_RAM_SIZE (1728)
#include <ACANFD_FeatherM4CAN.h>

LIS2DW12Sensor *Acc;

char version[] = "MCU Version 1.10";

#include "RS485.h"
#include <cppQueue.h>
#define	MAX_LENGTH		256


// For qspi
  //#include <SPI.h>c:\Users\aquar\OneDrive\文件\Arduino\libraries\Adafruit_SPIFlash\src\Adafruit_SPIFlash.h
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>
// for flashTransport definition
Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash flash(&flashTransport);

// For qspi
//#include "Adafruit_QSPI_Flash.h"
//Adafruit_QSPI_Flash flash;


// For SD Card
#define SD_Block_Size 512
#include <SD.h>
File2 myFile;
File2 myDir;
void printDirectory(File2 dir, int numTabs);
#define BUFF_SIZE 4096
uint8_t buffer[BUFF_SIZE];
int buff_index, sd_write_index, sd_read_index, sd_dir_index;
char filename[256];
char t_filename[256];


// Instantiate queue (with possible overwriting to ensure treatment of EOT_CHAR even if message longer than queue)
cppQueue	p(sizeof(char), MAX_LENGTH, FIFO, true);
cppQueue	q(sizeof(char), MAX_LENGTH, FIFO, true);
cppQueue	r(sizeof(char), MAX_LENGTH, FIFO, true);
//cppQueue	s(sizeof(char), MAX_LENGTH, FIFO, true);

//Ethernet
#include <OT2ITEth.h>

#define MAC_ADDRESS "\x02\x00\x00\x00\x00\x12"

uint8_t mac[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
OT2ITEth ot2itEth;

// #ifdef __cplusplus
// extern "C" {
// #endif

#include <stdio.h>

#include "lwip/tftp_client.h"
#include "lwip/tftp_server.h"
//#include "tftp_example.h"

#include <string.h>

/* Define a base directory for TFTP access
 * ATTENTION: This code does NOT check for sandboxing,
 * i.e. '..' in paths is not checked! */
#ifndef LWIP_TFTP_EXAMPLE_BASE_DIR
#define LWIP_TFTP_EXAMPLE_BASE_DIR ""
#endif

/* Define this to a file to get via tftp client */
#ifndef LWIP_TFTP_EXAMPLE_CLIENT_FILENAME
#define LWIP_TFTP_EXAMPLE_CLIENT_FILENAME "test.bin"
#endif

/* Define this to a server IP string */
#ifndef LWIP_TFTP_EXAMPLE_CLIENT_REMOTEIP
#define LWIP_TFTP_EXAMPLE_CLIENT_REMOTEIP "192.168.0.233"
#endif

#define TFTP_MAX_PAYLOAD_SIZE 512
#define MAX_FILE_SIZE 4096
#define PUT 1
#define GET 0

static char full_filename[256];
FILE* tftp_file = nullptr;
File2 tftp_sd_file;

int tftp_read_index, tftp_write_index;
uint8_t sd_file_buffer[MAX_FILE_SIZE];

void *tftp_open_file(const char* fname, u8_t is_write)
{
  int result;

  snprintf(full_filename, sizeof(full_filename), "%s%s", LWIP_TFTP_EXAMPLE_BASE_DIR, fname);
  full_filename[sizeof(full_filename)-1] = 0;

  if (!SD.begin(true)) 
  {
    Serial.println("SD card initialization failed!");        
    return 0;
  }
  Serial.println("SD card initialization successful!");

  memset(sd_file_buffer, 0, sizeof(sd_file_buffer));

  switch (is_write) {
  case PUT: // tftp put
    if (SD.exists(full_filename)) {
      SD.remove(full_filename);
    }
    tftp_sd_file = SD.open(full_filename, FILE_WRITE);
    tftp_sd_file.seek(0);
    tftp_write_index = 0;

    tftp_file = fmemopen((char*)sd_file_buffer, sizeof(sd_file_buffer), "w+");
    break;

  case GET: // tftp get
    if (!SD.exists(full_filename)) {
      Serial.println("File doesn't exist in SD card !");
      return 0;
    } else {
      tftp_read_index = 0;
    }
    tftp_sd_file = SD.open(full_filename);
    tftp_sd_file.seek(0);

    result = tftp_sd_file.read(&sd_file_buffer[0], MAX_FILE_SIZE);

    tftp_file = fmemopen((char*)sd_file_buffer, result, "r+");
    if (!tftp_file) {
      Serial.println("File cannot be created!");
      return 0;
    }
    tftp_sd_file.close();
    break;

  default:
    // Handle unexpected cases here
    break;
  }
  
  fflush(tftp_file);

  return (void*)tftp_file;
}

void* tftp_open(const char* fname, const char* mode, u8_t is_write)
{
  Serial.println("tftp_open called");
  LWIP_UNUSED_ARG(mode);
  
  return tftp_open_file(fname, is_write);
}

void tftp_close(void* handle)
{
  Serial.println("tftp_close called");
  fclose((FILE*)handle);
}

int tftp_read(void* handle, void* buf, int bytes)
{
  Serial.println("tftp_read called");

  int ret = fread(buf, 1, bytes, (FILE*)handle);
  //fseek((FILE*)handle, 512, SEEK_CUR);

  //Serial.println(ftell((FILE*)handle));
  //Serial.println(ret);

  tftp_read_index += ret;

  // if (bytes >= tftp_read_index)
  //   bytes = tftp_read_index;
  // else
  //   tftp_read_index -= bytes;

  if(tftp_read_index == 0)
  {
    Serial.println("No data in " + String(full_filename));
    return 0;
  }
  
  //Serial.println(tftp_read_index);

  return ret;
}

int tftp_write(void* handle, struct pbuf* p)
{
  Serial.println("tftp_write called");
  fclose((FILE*)handle);
  
  int ret = tftp_sd_file.write((uint8_t*)p->payload, p->len);
  tftp_write_index += ret;
  //tftp_sd_file.seek(ret);

  p = p->next;
  //Serial.println(ret);
  //Serial.println(tftp_write_index);

  
  if(ret < TFTP_MAX_PAYLOAD_SIZE || tftp_write_index == MAX_FILE_SIZE)
  {
    tftp_sd_file.close();
  }

  return ret;
}

/* For TFTP client only */
void tftp_error(void* handle, int err, const char* msg, int size)
{
  char message[100];

  LWIP_UNUSED_ARG(handle);

  memset(message, 0, sizeof(message));
  MEMCPY(message, msg, LWIP_MIN(sizeof(message)-1, (size_t)size));

  Serial.print("TFTP error:");
  Serial.print(err);
  Serial.println(message);
}

static const struct tftp_context tftp = {
  tftp_open,
  tftp_close,
  tftp_read,
  tftp_write,
  tftp_error
};

void ot2it_tftp_example_init_server(void)
{
  tftp_init_server(&tftp);
}

void ot2it_tftp_example_init_client(void)
{
  void *f;
  err_t err;
  ip_addr_t srv;
  int ret = ipaddr_aton(LWIP_TFTP_EXAMPLE_CLIENT_REMOTEIP, &srv);
  Serial.println("ipaddr_aton failed");

  err = tftp_init_client(&tftp);
  Serial.println("tftp_init_client failed");

  f = tftp_open_file(LWIP_TFTP_EXAMPLE_CLIENT_FILENAME, 1);
  if(f == NULL) {
    Serial.println("failed to create file");
  }
  err = tftp_get(f, &srv, TFTP_PORT, LWIP_TFTP_EXAMPLE_CLIENT_FILENAME, TFTP_MODE_OCTET);
  LWIP_ASSERT("tftp_get failed", err == ERR_OK);
}

// #ifdef __cplusplus
// }
// #endif

#define I_PI_OT2IT 1
//#define IO_Ex_Slave_Addr (0x3E)  //has been defined in board.h

//LIS2DW12 accelSensor( I2C_MODE, 0x18, SPISettings(0, MSBFIRST, SPI_MODE0) );
#define IO_Expander_DI_pin  26  
#define IO_Expander_DO_pin  29
#define IO_Expander_NRST_pin 7

// Digital Inputs
#define DI0_MCUIO   2  //PA06
#define DI1_MCUIO   3  //PA07
#define DI2_MCUIO   4  //PC07    
#define DI3_MCUIO   5  //PD10
#define DI4_MCUIO   6  //PC14
#define DI5_MCUIO   IO_Expander_DI_pin      //dummy pin 
#define DI6_MCUIO   (IO_Expander_DI_pin+1)  //dummy pin
#define DI7_MCUIO   (IO_Expander_DI_pin+2)  //dummy pin
#define DI7_Pulse_Pin 89 // PC02
// PUSH Button Keys
#define MCU_KEY_1   15  //PA22
#define MCU_KEY_2   16  //PA23
#define GEA_TOTAL_DIN_PINS  10

static byte dinPinArray[GEA_TOTAL_DIN_PINS] = {DI0_MCUIO, DI1_MCUIO, DI2_MCUIO, DI3_MCUIO, DI4_MCUIO, DI5_MCUIO, DI6_MCUIO, DI7_MCUIO, MCU_KEY_1, MCU_KEY_2};

// Digital Output
#define DO0_MCUIO   8  //PC10
#define DO1_MCUIO   9  //PD11
#define DO2_MCUIO   10  //PB14
#define DO3_MCUIO   11  //PC18
#define DO4_MCUIO   12  //PC17
#define DO5_MCUIO   IO_Expander_DO_pin      //
#define DO6_MCUIO   (IO_Expander_DO_pin+1)  //
#define DO7_MCUIO   (IO_Expander_DO_pin+2)  //

//Relay Output
#define RO0_MCUIO   51  //PC04
#define RO1_MCUIO   52  //PC05
#define RO2_MCUIO   53  //PC06

// LED Output
//#define MCU_LED_G   12  //D12
//#define MCU_LED_R   13  //D13
#define MCU_RGB_R   45  //PC28
#define MCU_RGB_G   46  //PC30
#define MCU_RGB_B   47  //PB31AII_AO_IN

#define MCU_RS485_TE 35 //PD20
#define CAN_STBY  50 //PC13
uint32_t RS485_Baud_Rate = 57600;

const uint8_t deviceID = 0;


#define GEA_TOTAL_DOUT_PINS   16 // DO - 8 + Relay Out - 3 + LED - 3

static byte ledPinArray[3] = {MCU_RGB_R, MCU_RGB_G, MCU_RGB_B};
static byte doutPinArray[GEA_TOTAL_DOUT_PINS] = {DO0_MCUIO, DO1_MCUIO, DO2_MCUIO, DO3_MCUIO, DO4_MCUIO, DO5_MCUIO, DO6_MCUIO, DO7_MCUIO,  
                                                 RO0_MCUIO, RO1_MCUIO, RO2_MCUIO, MCU_RGB_R, MCU_RGB_G, MCU_RGB_B};

//Analog Inputs

#define AII_AO_IN   A11  //A11
#define AII_A1_IN   A12  //A12
#define AIV0_IN     A10  //A10
#define AII_P1_IN   A9   //A9
#define AII_P2_IN   A7   //A7
#define AII_P3_IN   A8   //A8
#define AII_P0_IN   PIN_PCC_D1  //A3


#define GEA_TOTAL_AIN_PINS 7
static byte ainPinArray[GEA_TOTAL_AIN_PINS]={AII_AO_IN, AII_A1_IN, AIV0_IN, AII_P1_IN, AII_P2_IN, AII_P3_IN , AII_P0_IN};

#define ANALOG_OUT_0        A0
#define ANALOG_OUT_1        A1
#define ANALOG_OUT_0_Enable_Pin   32
#define ANALOG_OUT_1_Enable_Pin   33
#define GEA_TOTAL_AOUT_PINS 2
static byte aoutPinArray[GEA_TOTAL_AOUT_PINS]={ANALOG_OUT_0, ANALOG_OUT_1};

#define GEA_TOTAL_ANALOG_PINS (GEA_TOTAL_AIN_PINS + GEA_TOTAL_AOUT_PINS)

#define I2C_WRITE                   B00000000
#define I2C_READ                    B00001000
#define I2C_READ_CONTINUOUSLY       B00010000
#define I2C_STOP_READING            B00011000
#define I2C_READ_WRITE_MODE_MASK    B00011000
#define I2C_10BIT_ADDRESS_MODE_MASK B00100000
#define I2C_END_TX_MASK             B01000000
#define I2C_STOP_TX                 1
#define I2C_RESTART_TX              0
#define I2C_MAX_QUERIES             8
#define I2C_REGISTER_NOT_SPECIFIED  -1

// the minimum interval for sampling analog input
#define MINIMUM_SAMPLING_INTERVAL   1

#define SYSEX_TMP102_REQUEST        0x64 // 'd'
#define SYSEX_TMP102_REPLY          0x65 // 'e'
#define SYSEX_LED_BLINK             0x66 // 'f'
#define SYSEX_ACCEL_REQUEST         0x67 // 'g'
#define SYSEX_ACCEL_REPLY           0x68 // 'h'
#define SYSEX_COUNTER_REQUEST       0x62 //
#define SYSEX_COUNTER_REPLY         0x63
#define SYSEX_RTD_REQUEST           0x80
#define SYSEX_RTD_REPLY             0x81
#define SYSEX_485_REQUEST           0x82
#define SYSEX_485_REPLY             0x83
#define SYSEX_IOEXP_REQUEST         0x84
#define SYSEX_IOEXP_REPLY           0x85
#define SYSEX_CANBUS_REQUEST        0x86
#define SYSEX_CANBUS_REPLY          0x87
#define SYSEX_485_BAUDRATE_REQUEST  0x88
#define SYSEX_485_BAUDRATE_REPLY    0x89
#define SYSEX_FW_Version_REQUEST    0x8A
#define SYSEX_FW_Version_REPLY      0x8B
#define SYSEX_qspiFlash_REQUEST     0x8C
#define SYSEX_qspiFlash_REPLY       0x8D
#define SYSEX_SDCard_REQUEST        0x8E
#define SYSEX_SDCard_REPLY          0x8F
#define SYSEX_ResetCount_REQUEST    0x90
#define SYSEX_ResetCount_REPLY      0x91


#define MCU_IO_CH1_VOUT_EN          27
   
#define RTD0_SPI_RDY                 44
#define RTD0_SPI_MISO                80
#define RTD0_SPI_CLK                 82
#define RTD0_SPI_MOSI                81
#define RTD0_SPI_CS                  83

#define RTD1_SPI_RDY                 75
#define RTD1_SPI_MISO                41
#define RTD1_SPI_CLK                 69
#define RTD1_SPI_MOSI                37
#define RTD1_SPI_CS                  40

/*==============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

#ifdef FIRMATA_SERIAL_FEATURE
SerialFirmata serialFeature;
#endif

/* analog inputs */
int analogInputsToReport = 0; // bitwise array to store pin reporting

/* digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* pins configuration */
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else

/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
unsigned int samplingInterval = 19; // how often to run the main loop (in ms)

/* i2c data */
struct i2c_device_info {
  byte addr;
  int reg;
  byte bytes;
  byte stopTX;
};

/* for i2c read continuous more */
i2c_device_info query[I2C_MAX_QUERIES];

byte i2cRxData[256];
boolean isI2CEnabled = false;
signed char queryIndex = -1;
// default delay time between i2c read request and Wire.requestFrom()
unsigned int i2cReadDelayTime = 0;

Servo servos[MAX_SERVOS];
byte servoPinMap[TOTAL_PINS];
byte detachedServos[MAX_SERVOS];
byte detachedServoCount = 0;
byte servoCount = 0;

boolean isResetting = false;


byte IO_Expander_PortA_Value = 0;
byte IO_Expander_PortB_Value = 0;

//uint32_t DAC_Value[2] = {0,0};
int DAC_Value[2] = {0,0};

// Forward declare a few functions to avoid compiler errors with older versions
// of the Arduino IDE.
void setPinModeCallback(byte, int);
void reportAnalogCallback(byte analogPin, int value);
void sysexCallback(byte, byte, byte*);


// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      4000.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  1000.0

  RS485 rs485(&Serial3, MCU_RS485_TE, deviceID);
uint8_t ID;
uint8_t arr[32];
uint8_t len;

int pulse_cnt = 0;
int PULSE_COUNT = 0;

uint32_t spiFLASH_SIZE;
uint32_t power_reset_count = 1;

uint32_t merge_buffer(byte *input,byte *output,byte argc);

/* utility functions */
void wireWrite(byte data)
{
#if ARDUINO >= 100
  Wire.write((byte)data);
#else
  Wire.send(data);
#endif
}

byte wireRead(void)
{
#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}


/*==============================================================================
 * FUNCTIONS
 *============================================================================*/
void pulse_counter(void)
{
  pulse_cnt++; 
  IO_Expander_Write_Reg(0x19,0xFF);
}
void RS485_Init()
{
  Serial3.begin(RS485_Baud_Rate); // This is for RS485
  //pinPeripheral(1, PIO_SERCOM); //Assign RX function to pin 1
  //pinPeripheral(0, PIO_SERCOM); //Assign TX function to pin 0
}
void CANBus_Init()
{
  // ArbitrationBitRate = 500k
  // DataBitRate = 500 X 4 = 2M
  ACANFD_FeatherM4CAN_Settings settings (500 * 1000, DataBitRateFactor::x4);
  settings.mModuleMode = ACANFD_FeatherM4CAN_Settings::NORMAL_FD;
  const uint32_t errorCode = can1.beginFD (settings);
  
  if (0 == errorCode) {
    Serial.println ("can configuration ok") ;
  }else{
    Serial.print ("Error can configuration: 0x") ;
    Serial.println (errorCode, HEX) ;
  }

}
void initGEAIOs(void)
{
    // Digital Inputs
    //pinMode(DI0_MCUIO, INPUT);
    pinMode(DI1_MCUIO, INPUT_PULLUP);
    pinMode(DI2_MCUIO, INPUT_PULLUP);
    pinMode(DI3_MCUIO, INPUT_PULLUP);
    pinMode(DI4_MCUIO, INPUT_PULLUP);
    pinMode(DI5_MCUIO, INPUT_PULLUP);
    pinMode(DI6_MCUIO, INPUT_PULLUP);
    pinMode(DI7_MCUIO, INPUT_PULLUP);
    pinMode(DI7_Pulse_Pin, INPUT_PULLUP);

    // PUSH Button Keys
    pinMode(MCU_KEY_1, INPUT);
    pinMode(MCU_KEY_2, INPUT);

    // Digital Output
    pinMode(DO0_MCUIO, OUTPUT);
    pinMode(DO1_MCUIO, OUTPUT);
    pinMode(DO2_MCUIO, OUTPUT);
    pinMode(DO3_MCUIO, OUTPUT);
    pinMode(DO4_MCUIO, OUTPUT);
    pinMode(DO5_MCUIO, OUTPUT);
    pinMode(DO6_MCUIO, OUTPUT);
    pinMode(DO7_MCUIO, OUTPUT);

    //Relay Output
    pinMode(RO0_MCUIO, OUTPUT);
    pinMode(RO1_MCUIO, OUTPUT);
    pinMode(RO2_MCUIO, OUTPUT);

    // LED Output
  //  pinMode(MCU_LED_G, OUTPUT);
  //  pinMode(MCU_LED_R, OUTPUT);
    pinMode(MCU_RGB_R, OUTPUT);
    pinMode(MCU_RGB_G, OUTPUT);
    pinMode(MCU_RGB_B, OUTPUT);

    //Analog Inputs
    pinMode(AII_AO_IN, PIN_MODE_ANALOG);
    pinMode(AII_A1_IN, PIN_MODE_ANALOG);
    pinMode(AIV0_IN, PIN_MODE_ANALOG);
    pinMode(AII_P1_IN, PIN_MODE_ANALOG);
    pinMode(AII_P2_IN, PIN_MODE_ANALOG);
    pinMode(AII_P3_IN, PIN_MODE_ANALOG);
    pinMode(AII_P0_IN, PIN_MODE_ANALOG);
   

    //Analog Output
    pinMode(ANALOG_OUT_0_Enable_Pin,OUTPUT);
    pinMode(ANALOG_OUT_1_Enable_Pin,OUTPUT);
    pinMode(ANALOG_OUT_0, PIN_MODE_ANALOG);
    pinMode(ANALOG_OUT_1, PIN_MODE_ANALOG);
 
    analogReference(AR_INTERNAL2V5);
    DAC->CTRLB.reg = DAC_CTRLB_REFSEL_INTREF;
    analogReadResolution(12);
    digitalWrite(ANALOG_OUT_1_Enable_Pin, HIGH);
    analogWrite(ANALOG_TO_PIN(0),  DAC_Value[0]);
    analogWrite(ANALOG_TO_PIN(1),  DAC_Value[1]);
    digitalWrite(ANALOG_OUT_0_Enable_Pin, HIGH);
    digitalWrite(ANALOG_OUT_1_Enable_Pin, HIGH);
    analogWrite(ANALOG_TO_PIN(0),  DAC_Value[0]);
    analogWrite(ANALOG_TO_PIN(1),  DAC_Value[1]);

    pinMode(MCU_IO_CH1_VOUT_EN, OUTPUT);
    digitalWrite(MCU_IO_CH1_VOUT_EN, HIGH);

    pinMode(RTD0_SPI_RDY, OUTPUT);
    pinMode(RTD1_SPI_RDY, OUTPUT);

    pinMode(IO_Expander_NRST_pin, OUTPUT);
    

    RS485_Init();
    enableI2CPins();

    IO_Exapnader_Init();
    CANBus_Init();

    /* Attach to interrupt for pulse counting */
    //attachInterrupt(digitalPinToInterrupt(DI7_Pulse_Pin), pulse_counter, RISING);
    attachInterrupt(digitalPinToInterrupt(DI7_Pulse_Pin), pulse_counter, FALLING);
}

void attachServo(byte pin, int minPulse, int maxPulse)
{
  if (servoCount < MAX_SERVOS) {
    // reuse indexes of detached servos until all have been reallocated
    if (detachedServoCount > 0) {
      servoPinMap[pin] = detachedServos[detachedServoCount - 1];
      if (detachedServoCount > 0) detachedServoCount--;
    } else {
      servoPinMap[pin] = servoCount;
      servoCount++;
    }
    if (minPulse > 0 && maxPulse > 0) {
      servos[servoPinMap[pin]].attach(PIN_TO_DIGITAL(pin), minPulse, maxPulse);
    } else {
      servos[servoPinMap[pin]].attach(PIN_TO_DIGITAL(pin));
    }
  } else {
    Firmata.sendString("Max servos attached");
  }
}

void detachServo(byte pin)
{
  servos[servoPinMap[pin]].detach();
  // if we're detaching the last servo, decrement the count
  // otherwise store the index of the detached servo
  if (servoPinMap[pin] == servoCount && servoCount > 0) {
    servoCount--;
  } else if (servoCount > 0) {
    // keep track of detached servos because we want to reuse their indexes
    // before incrementing the count of attached servos
    detachedServoCount++;
    detachedServos[detachedServoCount - 1] = servoPinMap[pin];
  }

  servoPinMap[pin] = 255;
}

void enableI2CPins()
{
  byte i;
  // is there a faster way to do this? would probaby require importing
  // Arduino.h to get SCL and SDA pins
  for (i = 0; i < TOTAL_PINS; i++) {
    if (IS_PIN_I2C(i)) {
      // mark pins as i2c so they are ignore in non i2c data requests
      setPinModeCallback(i, PIN_MODE_I2C);
    }
  }

  isI2CEnabled = true;
  Wire.setClock(400000);
  Wire.begin();
}

/* disable the i2c pins so they can be used for other functions */
void disableI2CPins() {
  isI2CEnabled = false;
  // disable read continuous mode for all devices
  queryIndex = -1;
}

void readAndReportData(byte address, int theRegister, byte numBytes, byte stopTX) {
  // allow I2C requests that don't require a register read
  // for example, some devices using an interrupt pin to signify new data available
  // do not always require the register read so upon interrupt you call Wire.requestFrom()
  if (theRegister != I2C_REGISTER_NOT_SPECIFIED) {
    Wire.beginTransmission(address);
    wireWrite((byte)theRegister);
    Wire.endTransmission(stopTX); // default = true
    // do not set a value of 0
    if (i2cReadDelayTime > 0) {
      // delay is necessary for some devices such as WiiNunchuck
      delayMicroseconds(i2cReadDelayTime);
    }
  } else {
    theRegister = 0;  // fill the register with a dummy value
  }

  Wire.requestFrom(address, numBytes);  // all bytes are returned in requestFrom

  // check to be sure correct number of bytes were returned by slave
  if (numBytes < Wire.available()) {
    Firmata.sendString("I2C: Too many bytes received");
  } else if (numBytes > Wire.available()) {
    Firmata.sendString("I2C: Too few bytes received");
  }

  i2cRxData[0] = address;
  i2cRxData[1] = theRegister;

  for (int i = 0; i < numBytes && Wire.available(); i++) {
    i2cRxData[2 + i] = wireRead();
  }

  // send slave address, register and received bytes
  Firmata.sendSysex(SYSEX_I2C_REPLY, numBytes + 2, i2cRxData);
  //Firmata.sendSysex(SYSEX_I2C_REPLY, 2, i2cRxData);
}

void outputPort(byte portNumber, byte portValue, byte forceSend)
{
  // pins not configured as INPUT are cleared to zeros
  portValue = portValue & portConfigInputs[portNumber];
  // only send if the value is different than previously sent
  if (forceSend || previousPINs[portNumber] != portValue) {
    Firmata.sendDigitalPort(portNumber, portValue);
    previousPINs[portNumber] = portValue;
  }
}

/* -----------------------------------------------------------------------------
 * check all the active digital inputs for change of state, then add any events
 * to the Serial output queue using Serial.print() */
void checkDigitalInputs(void)
{

  /* Using non-looping code allows constants to be given to readPort().
   * The compiler will apply substantial optimizations if the inputs
   * to readPort() are compile-time constants. */
  if (TOTAL_PORTS > 0 && reportPINs[0]) outputPort(0, readPort(0, portConfigInputs[0]), false);
  if (TOTAL_PORTS > 1 && reportPINs[1]) outputPort(1, readPort(1, portConfigInputs[1]), false);
  if (TOTAL_PORTS > 2 && reportPINs[2]) outputPort(2, readPort(2, portConfigInputs[2]), false);
  if (TOTAL_PORTS > 3 && reportPINs[3]) outputPort(3, readPort(3, portConfigInputs[3]), false);
  if (TOTAL_PORTS > 4 && reportPINs[4]) outputPort(4, readPort(4, portConfigInputs[4]), false);
  if (TOTAL_PORTS > 5 && reportPINs[5]) outputPort(5, readPort(5, portConfigInputs[5]), false);
  if (TOTAL_PORTS > 6 && reportPINs[6]) outputPort(6, readPort(6, portConfigInputs[6]), false);
  if (TOTAL_PORTS > 7 && reportPINs[7]) outputPort(7, readPort(7, portConfigInputs[7]), false);
  if (TOTAL_PORTS > 8 && reportPINs[8]) outputPort(8, readPort(8, portConfigInputs[8]), false);
  if (TOTAL_PORTS > 9 && reportPINs[9]) outputPort(9, readPort(9, portConfigInputs[9]), false);
  if (TOTAL_PORTS > 10 && reportPINs[10]) outputPort(10, readPort(10, portConfigInputs[10]), false);
  if (TOTAL_PORTS > 11 && reportPINs[11]) outputPort(11, readPort(11, portConfigInputs[11]), false);
  if (TOTAL_PORTS > 12 && reportPINs[12]) outputPort(12, readPort(12, portConfigInputs[12]), false);
  if (TOTAL_PORTS > 13 && reportPINs[13]) outputPort(13, readPort(13, portConfigInputs[13]), false);
  if (TOTAL_PORTS > 14 && reportPINs[14]) outputPort(14, readPort(14, portConfigInputs[14]), false);
  if (TOTAL_PORTS > 15 && reportPINs[15]) outputPort(15, readPort(15, portConfigInputs[15]), false);
}

// -----------------------------------------------------------------------------
/* sets the pin mode to the correct state and sets the relevant bits in the
 * two bit-arrays that track Digital I/O and PWM status
 */
void setPinModeCallback(byte pin, int mode)
{
  if (Firmata.getPinMode(pin) == PIN_MODE_IGNORE)
    return;

  if (Firmata.getPinMode(pin) == PIN_MODE_I2C && isI2CEnabled && mode != PIN_MODE_I2C) {
    // disable i2c so pins can be used for other functions
    // the following if statements should reconfigure the pins properly
    disableI2CPins();
  }
  if (IS_PIN_DIGITAL(pin) && mode != PIN_MODE_SERVO) {
    if (servoPinMap[pin] < MAX_SERVOS && servos[servoPinMap[pin]].attached()) {
      detachServo(pin);
    }
  }
  if (IS_PIN_ANALOG(pin)) {
    reportAnalogCallback(PIN_TO_ANALOG(pin), mode == PIN_MODE_ANALOG ? 1 : 0); // turn on/off reporting
  }
  if (IS_PIN_DIGITAL(pin)) {
    if (mode == INPUT || mode == PIN_MODE_PULLUP) {
      portConfigInputs[pin / 8] |= (1 << (pin & 7));
    } else {
      portConfigInputs[pin / 8] &= ~(1 << (pin & 7));
    }
  }
  Firmata.setPinState(pin, 0);
  switch (mode) {
    case PIN_MODE_ANALOG:
      if (IS_PIN_ANALOG(pin)) {
        if (IS_PIN_DIGITAL(pin)) {
          pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
#if ARDUINO <= 100
          // deprecated since Arduino 1.0.1 - TODO: drop support in Firmata 2.6
          digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
#endif
        }
        Firmata.setPinMode(pin, PIN_MODE_ANALOG);
      }
      break;
    case INPUT:
//      Serial3.printf("INPUT\n");
//      Serial3.flush();
     if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
#if ARDUINO <= 100
        // deprecated since Arduino 1.0.1 - TODO: drop support in Firmata 2.6
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
#endif
        Firmata.setPinMode(pin, INPUT);
      }
      break;
    case PIN_MODE_PULLUP:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT_PULLUP);
        Firmata.setPinMode(pin, PIN_MODE_PULLUP);
        Firmata.setPinState(pin, 1);
      }
      break;
    case OUTPUT:
      //Serial1.printf("OUTPUT\r\n");
      if (IS_PIN_DIGITAL(pin)) {
        if (Firmata.getPinMode(pin) == PIN_MODE_PWM) {
          // Disable PWM if pin mode was previously set to PWM.
          //digitalWrite(PIN_TO_DIGITAL(pin), LOW);
          //digitalWrite(27, HIGH);
        }
        pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
        Firmata.setPinMode(pin, OUTPUT);
      }
      break;
    case PIN_MODE_PWM:
      if (IS_PIN_PWM(pin)) {
        pinMode(PIN_TO_PWM(pin), OUTPUT);
        analogWrite(PIN_TO_PWM(pin), 0);
        Firmata.setPinMode(pin, PIN_MODE_PWM);
      }
      break;
    case PIN_MODE_SERVO:
      if (IS_PIN_DIGITAL(pin)) {
        Firmata.setPinMode(pin, PIN_MODE_SERVO);
        if (servoPinMap[pin] == 255 || !servos[servoPinMap[pin]].attached()) {
          // pass -1 for min and max pulse values to use default values set
          // by Servo library
          attachServo(pin, -1, -1);
        }
      }
      break;
    case PIN_MODE_I2C:
      if (IS_PIN_I2C(pin)) {
        // mark the pin as i2c
        // the user must call I2C_CONFIG to enable I2C for a device
        Firmata.setPinMode(pin, PIN_MODE_I2C);
      }
      break;
    case PIN_MODE_SERIAL:
#ifdef FIRMATA_SERIAL_FEATURE
      serialFeature.handlePinMode(pin, PIN_MODE_SERIAL);
#endif
      break;
    default:
      Firmata.sendString("Unknown pin mode"); // TODO: put error msgs in EEPROM
  }
  // TODO: save status to EEPROM here, if changed
}

/*
 * Sets the value of an individual pin. Useful if you want to set a pin value but
 * are not tracking the digital port state.
 * Can only be used on pins configured as OUTPUT.
 * Cannot be used to enable pull-ups on Digital INPUT pins.
 */
void setPinValueCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS && IS_PIN_DIGITAL(pin)) {
    if (Firmata.getPinMode(pin) == OUTPUT) {
      Firmata.setPinState(pin, value);
      digitalWrite(PIN_TO_DIGITAL(pin), value);
    }
  }
}
// This is DAC function
void analogWriteCallback(byte pin, int value)
{
  //Serial1.printf("analogWrite, pin=%d, a_pin=%d, value=%d\r\n",pin,ANALOG_TO_PIN(pin),value);
  if (pin < TOTAL_PINS) {
    switch (Firmata.getPinMode(pin)) {
      default:
        if(pin < GEA_TOTAL_AOUT_PINS){
        //Serial1.printf("analogWrite, pin=%d, a_pin=%d, value=%d\r\n",pin,ANALOG_TO_PIN(pin),value);
          DAC_Value[pin] = value; 
          for(int(i) = 0 ;i<GEA_TOTAL_AOUT_PINS; i++ ){
            digitalWrite(ANALOG_OUT_0_Enable_Pin, HIGH);
            digitalWrite(ANALOG_OUT_1_Enable_Pin, HIGH);
            analogWrite(ANALOG_TO_PIN(GEA_TOTAL_AOUT_PINS-i-1), DAC_Value[GEA_TOTAL_AOUT_PINS-i-1]);
          }
        }
        break;
      case PIN_MODE_SERVO:
        if (IS_PIN_DIGITAL(pin))
          servos[servoPinMap[pin]].write(value);
        Firmata.setPinState(pin, value);
        break;
      case PIN_MODE_PWM:
        if (IS_PIN_PWM(pin)) {
          analogWrite(ANALOG_TO_PIN(pin), value);
          //analogWrite(PIN_TO_PWM(pin), value);
        }
        Firmata.setPinState(ANALOG_TO_PIN(pin), value);
        break;
    }
  }
}

void digitalWriteCallback(byte port, int value)
{
  byte pin, lastPin, pinValue, mask = 1, pinWriteMask = 0;

  if (port < TOTAL_PORTS) {
    // create a mask of the pins on this port that are writable.
    lastPin = port * 8 + 8;
    if (lastPin > TOTAL_PINS) lastPin = TOTAL_PINS;
    for (pin = port * 8; pin < lastPin; pin++) {
      // do not disturb non-digital pins (eg, Rx & Tx)
      if (IS_PIN_DIGITAL(pin)) {
        // do not touch pins in PWM, ANALOG, SERVO or other modes
        if (Firmata.getPinMode(pin) == OUTPUT || Firmata.getPinMode(pin) == INPUT) {
          pinValue = ((byte)value & mask) ? 1 : 0;
          if (Firmata.getPinMode(pin) == OUTPUT) {
            //if(value & mask){
              pinWriteMask |= mask;
            //}
          } else if (Firmata.getPinMode(pin) == INPUT && pinValue == 1 && Firmata.getPinState(pin) != 1) {
            // only handle INPUT here for backwards compatibility
#if ARDUINO > 100
            pinMode(pin, INPUT_PULLUP);
#else
            // only write to the INPUT pin to enable pullups if Arduino v1.0.0 or earlier
            pinWriteMask |= mask;
#endif
          }
          Firmata.setPinState(pin, pinValue);
        }
      }
      mask = mask << 1;
    }
    writePort(port, (byte)value, pinWriteMask);
  }
}


// -----------------------------------------------------------------------------
/* sets bits in a bit array (int) to toggle the reporting of the analogIns
 */
//void FirmataClass::setAnalogPinReporting(byte pin, byte state) {
//}
// This funcation is called when host set a analog pin to read
void reportAnalogCallback(byte analogPin, int value)
{
  //Serial1.print("Call reportAnalog\r\n");  //digitalWrite(MCU_LED_R, HIGH);
  if (analogPin < TOTAL_ANALOG_PINS) 
  {
    if (value == 0) {
      analogInputsToReport = analogInputsToReport & ~ (1 << analogPin);
    } else 
    {
      analogInputsToReport = analogInputsToReport | (1 << analogPin);
      // prevent during system reset or all analog pin values will be reported
      // which may report noise for unconnected analog pins
      if (!isResetting) 
      {
        // Send pin value immediately. This is helpful when connected via
        // ethernet, wi-fi or bluetooth so pin states can be known upon
        // reconnecting.
        //Serial1.printf("AO=%d\r\n",analogPin);
        Firmata.sendAnalog(analogPin, analogRead(ANALOG_TO_PIN(analogPin)));
      }
    }
  }
  // TODO: save status to EEPROM here, if changed
}

void reportDigitalCallback(byte port, int value)
{
  if (port < TOTAL_PORTS) {
    reportPINs[port] = (byte)value;
    // Send port value immediately. This is helpful when connected via
    // ethernet, wi-fi or bluetooth so pin states can be known upon
    // reconnecting.
    if (value) outputPort(port, readPort(port, portConfigInputs[port]), true);
  }
  // do not disable analog reporting on these 8 pins, to allow some
  // pins used for digital, others analog.  Instead, allow both types
  // of reporting to be enabled, but check if the pin is configured
  // as analog when sampling the analog inputs.  Likewise, while
  // scanning digital pins, portConfigInputs will mask off values from any
  // pins configured as analog
}

#define TMP102_I2C_ADDRESS 72
#define IO_EXPANDER_I2C_ADDR 0x3E

#if 1
void readAndReportCounterData()
{
  int freq;
  pulse_cnt = 0;
  PULSE_COUNT = 0;
  //zerotimer.enable(true);
  delay(100);
  PULSE_COUNT = pulse_cnt;
  //zerotimer.enable(false);
  i2cRxData[0] = (PULSE_COUNT) & 0x7F;
  i2cRxData[1] = (PULSE_COUNT >> 7) & 0x7F;
  i2cRxData[2] = (PULSE_COUNT >> 14) & 0x7F;
  i2cRxData[3] = (PULSE_COUNT >> 21) & 0x7F;
  //Serial1.print(frequency);
  //Serial1.print(" Hz\r\n");

  // send slave address, register and received bytes
  Firmata.sendSysex(SYSEX_COUNTER_REPLY, 4, i2cRxData);  
  //Serial1.print(frequency);
  //Serial1.print(" Hz\r\n");
}
#else
void readAndReportCounterData()
{
  i2cRxData[0] = (pulse_cnt) & 0x7F;
  i2cRxData[1] = (pulse_cnt >> 7) & 0x7F;
  i2cRxData[2] = (pulse_cnt >> 14) & 0x7F;
  i2cRxData[3] = (pulse_cnt >> 21) & 0x7F;

  // send slave address, register and received bytes
  Firmata.sendSysex(SYSEX_COUNTER_REPLY, 4, i2cRxData);  
}
#endif



void readAndReportAccelData()
{
  uint16_t value;
  value = 0;
  int x,y,z;

  int32_t accelerometer[3];
  memset(accelerometer, 0, 3);
  
  DEV_I2C.begin();

  Acc=new LIS2DW12Sensor (&DEV_I2C);
  Acc->Enable_X();
  Acc->Get_X_Axes(accelerometer);
  x = accelerometer[0];
  y = accelerometer[1];
  z = accelerometer[2];


  i2cRxData[0] = x & 0x7F;
  i2cRxData[1] = (x >> 7) & 0x7F;
  i2cRxData[2] = (x >> 14) & 0x02;
  i2cRxData[3] = y & 0x7F;
  i2cRxData[4] = (y >> 7) & 0x7F;
  i2cRxData[5] = (y >> 14) & 0x02;
  i2cRxData[6] = z & 0x7F;
  i2cRxData[7] = (z >> 7) & 0x7F;
  i2cRxData[8] = (z >> 14) & 0x02;


  // send slave address, register and received bytes
  Firmata.sendSysex(SYSEX_ACCEL_REPLY, 9, i2cRxData);
}

void readAndReportRTD(int rtd_no)
{
  float temp;
  short int CS,DI,DO,CLK;
  int temp_value;
  
  if(rtd_no == 0){  // This one is same as gea
    CS = RTD0_SPI_CS;
    DI = RTD0_SPI_MOSI;
    DO = RTD0_SPI_MISO;
    CLK = RTD0_SPI_CLK;
    //digitalWrite(RTD0_SPI_RDY, LOW);
  }
  else if(rtd_no == 1){ 
    CS = RTD1_SPI_CS;
    DI = RTD1_SPI_MOSI;
    DO = RTD1_SPI_MISO;
    CLK = RTD1_SPI_CLK;
    //digitalWrite(RTD1_SPI_RDY, LOW);
  }
  else
  {
    return;
  }
  Adafruit_MAX31865 thermo = Adafruit_MAX31865(CS, DI, DO, CLK);
  thermo.begin(MAX31865_2WIRE);
  uint16_t rtd = thermo.readRTD();
  float ratio = rtd;
  ratio /= 32768;
  //Serial1.printf("RTD no = %d\r\n",rtd_no);
  //Serial1.print("Ratio = "); Serial1.println(ratio,8);
  //Serial1.print("Resistance = "); Serial1.println(RREF*ratio,8);
  //Serial1.print("Temperature = "); Serial1.println(thermo.temperature(RNOMINAL, RREF));
  temp = thermo.temperature(RNOMINAL, RREF);
  temp_value = temp*100;
  i2cRxData[0] = temp_value & 0x7F;
  i2cRxData[1] = (temp_value >> 7) & 0x7F;
  i2cRxData[2] = (temp_value >> 14) & 0x7F;
  i2cRxData[3] = (temp_value >> 21) & 0x7F;
  if(rtd_no == 0){  // This one is same as gea
    digitalWrite(RTD0_SPI_RDY, HIGH);
  }
  else if(rtd_no == 1){ 
    digitalWrite(RTD1_SPI_RDY, HIGH);
  }
  
  Firmata.sendSysex(SYSEX_RTD_REPLY, 4, i2cRxData);
}


void readAndReportTMP102Data()
{
  float temperature;
  int temp_value;
  /*Reset the register pointer (by default it is ready to read temperatures)
    You can alter it to a writeable register and alter some of the configuration - 
    the sensor is capable of alerting you if the temperature is above or below a specified threshold. */
  Wire.begin(); //Join I2C Bus   
  /* The TMP102 uses the default settings with the address 0x48 using Wire.

     Optionally, if the address jumpers are modified, or using a different I2C bus,
     these parameters can be changed here. E.g. sensor0.begin(0x49,Wire1)

     It will return true on success or false on failure to communicate. */
  sensor0.begin();

  delay(100);
  sensor0.wakeup();
  temperature = sensor0.readTempC();
  sensor0.sleep();
  temp_value = temperature*100;
 
  i2cRxData[0] = temp_value & 0x7F;
  i2cRxData[1] = (temp_value >> 7) & 0x7F;
  i2cRxData[2] = (temp_value >> 14) & 0x7F;
  // send slave address, register and received bytes
  Firmata.sendSysex(SYSEX_TMP102_REPLY, 3, i2cRxData);

}
void readRS485Data()
{
int count=0,i;
  //rs485.send(1, (uint8_t *)msg, strlen(msg));
  for(i=0;i<5;i++){
	  while (rs485.available())					// Ensure Serial is available
	  {
	    char rcv = (char) rs485.read();	// Read char from Serial
      //Serial1.print(rcv);

		  if (rcv != -1)						// If no character received: -1 (do not push)
		  {
		 	  q.push(&rcv);					// Push char to cppQueue
		  }
	  }
    delayMicroseconds(10);
  }
}
void readDebugData()
{
int count=0,i;
  //rs485.send(1, (uint8_t *)msg, strlen(msg));
  for(i=0;i<5;i++){
	  while (Serial1.available())					// Ensure Serial is available
	  {
	    char rcv = (char) Serial1.read();	// Read char from Serial

		  if (rcv != -1)						// If no character received: -1 (do not push)
		  {
		 	  p.push(&rcv);					// Push char to cppQueue
		  }
	  }
    delayMicroseconds(10);
  }
}
void readRS485Data_return()
{
int count=0,i;
  //rs485.send(1, (uint8_t *)msg, strlen(msg));
  for(i=0;i<5;i++){
	  while (rs485.available())					// Ensure Serial is available
	  {
	    char rcv = (char) rs485.read();	// Read char from Serial

		  if (rcv != -1)						// If no character received: -1 (do not push)
		  {
		 	  q.push(&rcv);					// Push char to cppQueue
		  }
	  }
    delayMicroseconds(10);
  }
	if (!q.isEmpty())				// Only if q is not empty
	{
	  char snd;
    int i=0;
    count = +q.getCount();
    for(i=0;i<count;i++){
	   q.pop(&snd);		// While characters are available in q
	   i2cRxData[i]= snd;
	 }
	}
  //i2cRxData[0] = 65;
  //i2cRxData[1] = 66;
  //i2cRxData[2] = 67;
  //count = 3;
  //i2cRxData[0] = count;
  Firmata.sendSysex(SYSEX_485_REPLY, count, i2cRxData);
}

void readRS485BaudRate_return()
{
int count=0;

  i2cRxData[0] = RS485_Baud_Rate & 0x7F;
  i2cRxData[1] = (RS485_Baud_Rate >> 7) & 0x7F;
  i2cRxData[2] = (RS485_Baud_Rate >> 14) & 0x7F;
  i2cRxData[3] = (RS485_Baud_Rate >> 21) & 0x7F;

  //Serial1.println(i2cRxData[0]);
  //Serial1.println(i2cRxData[1]);
  //Serial1.println(i2cRxData[2]);
  //Serial1.println(i2cRxData[3]);
  count = 4;
  //i2cRxData[0] = count;
  Firmata.sendSysex(SYSEX_485_BAUDRATE_REPLY, count, i2cRxData);  
}


      //rs485.read();
      //rs485.receive(ID, arr, len);
//-------------------------------------------------------------------------------
// CANBus function
static const uint32_t PERIOD = 500 ;
static uint32_t gSentCount = 0 ;
/*
void SendCANBusData()
{
  CANFDMessage frame;
  frame.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH ;
  frame.id = 0x12345678 ;
  frame.len = 8 ;
  for (uint8_t i=0 ; i<frame.len ; i++) {
    frame.data [i] = i ;
  }

  const uint32_t sendStatus = can1.tryToSendReturnStatusFD (frame);
  if (sendStatus == 0) {
      gSentCount += 1;
      Serial.print ("Sent ") ;
      Serial.println (gSentCount);
    }else{
      Serial.println ("Buffer Full");
    }
}
*/
void SendCANBusData(unsigned char *data , int len)
{
  CANFDMessage frame;
  frame.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH ;
  frame.id = (data[0] << 8) + data[1] ;
  frame.len = len -2;

  for (uint8_t i=0 ; i< frame.len ; i++) {
    frame.data [i] = data[i+2];
  }
  frame.pad();
  const uint32_t sendStatus = can1.tryToSendReturnStatusFD (frame);
  if (sendStatus == 0) {
      gSentCount += 1;
      //Serial.print ("Sent ") ;
      Serial.println (gSentCount);
    }else{
      //Serial.println ("Buffer Full");
    }
}

/*
void ReadCANBusData()
{
  CANFDMessage frame ;
    if (can1.receiveFD0 (frame)) { 
      Serial.print ("Received id: ");
      Serial.println(frame.id);
    }
}
*/
void ReadCANBusData()
{
  CANFDMessage frame ;
  int len = (char) frame.len;	// Read char from Serial
  int i;

	while (can1.receiveFD0 (frame))					// Ensure Serial is available
	{
    //Serial1.print(frame.id);
    byte B;
    B= (frame.id >> 8);
    r.push(&B);
    B= (frame.id & 0xFF);
    r.push(&B);
    len = (char) frame.len;
    for(i=0;i<len;i++)
		  r.push(&frame.data[i]);					// Push char to cppQueue
	}
}
void ReadCANBusData_return()
{
int count=0,i;
  //rs485.send(1, (uint8_t *)msg, strlen(msg));
  ReadCANBusData();
	if (!r.isEmpty())				// Only if q is not empty
	{
	  char snd;
    int i=0;
    count = +r.getCount();
    for(i=0;i<count;i++){
	   r.pop(&snd);		// While characters are available in q
	   i2cRxData[i]= snd;
	 }
	}
  Firmata.sendSysex(SYSEX_CANBUS_REPLY, count, i2cRxData);
}
//-------------------------------------------------------------------------------

/*==============================================================================
 * SYSEX-BASED commands
 *============================================================================*/
void sysexCallback(byte command, byte argc, byte *argv)
{
  byte mode;
  byte stopTX;
  byte slaveAddress;
  byte data;
  int slaveRegister;
  unsigned int delayTime;
  uint16_t count;
  uint32_t id,index;
  uint8_t result;
  int function;
  uint32_t number;
  uint32_t address;
  uint32_t size;

  switch (command) {
    case I2C_REQUEST:
      mode = argv[1] & I2C_READ_WRITE_MODE_MASK;
      if (argv[1] & I2C_10BIT_ADDRESS_MODE_MASK) {
        Firmata.sendString("10-bit addressing not supported");
        return;
      }
      else {
        slaveAddress = argv[0];
      }

      // need to invert the logic here since 0 will be default for client
      // libraries that have not updated to add support for restart tx
      if (argv[1] & I2C_END_TX_MASK) {
        stopTX = I2C_RESTART_TX;
      }
      else {
        stopTX = I2C_STOP_TX; // default
      }

      switch (mode) {
        case I2C_WRITE:
          Wire.beginTransmission(slaveAddress);
          for (byte i = 2; i < argc; i += 2) {
            data = argv[i] + (argv[i + 1] << 7);
            wireWrite(data);
          }
          Wire.endTransmission();
          delayMicroseconds(70);
          break;
        case I2C_READ:
          if (argc == 6) {
            // a slave register is specified
            slaveRegister = argv[2] + (argv[3] << 7);
            data = argv[4] + (argv[5] << 7);  // bytes to read
            //Firmata.sendSysex(SYSEX_I2C_REPLY, argc, argv);
          }
          else {
            // a slave register is NOT specified
            slaveRegister = I2C_REGISTER_NOT_SPECIFIED;
            data = argv[2] + (argv[3] << 7);  // bytes to read
            //Firmata.sendSysex(SYSEX_I2C_REPLY, 4, argv);
            break;
          }
          readAndReportData(slaveAddress, (int)slaveRegister, data, stopTX);
          break;
        case I2C_READ_CONTINUOUSLY:
          if ((queryIndex + 1) >= I2C_MAX_QUERIES) {
            // too many queries, just ignore
            Firmata.sendString("too many queries");
            break;
          }
          if (argc == 6) {
            // a slave register is specified
            slaveRegister = argv[2] + (argv[3] << 7);
            data = argv[4] + (argv[5] << 7);  // bytes to read
          }
          else {
            // a slave register is NOT specified
            slaveRegister = (int)I2C_REGISTER_NOT_SPECIFIED;
            data = argv[2] + (argv[3] << 7);  // bytes to read
          }
          queryIndex++;
          query[queryIndex].addr = slaveAddress;
          query[queryIndex].reg = slaveRegister;
          query[queryIndex].bytes = data;
          query[queryIndex].stopTX = stopTX;
          break;
        case I2C_STOP_READING:
          byte queryIndexToSkip;
          // if read continuous mode is enabled for only 1 i2c device, disable
          // read continuous reporting for that device
          if (queryIndex <= 0) {
            queryIndex = -1;
          } else {
            queryIndexToSkip = 0;
            // if read continuous mode is enabled for multiple devices,
            // determine which device to stop reading and remove it's data from
            // the array, shifiting other array data to fill the space
            for (byte i = 0; i < queryIndex + 1; i++) {
              if (query[i].addr == slaveAddress) {
                queryIndexToSkip = i;
                break;
              }
            }

            for (byte i = queryIndexToSkip; i < queryIndex + 1; i++) {
              if (i < I2C_MAX_QUERIES) {
                query[i].addr = query[i + 1].addr;
                query[i].reg = query[i + 1].reg;
                query[i].bytes = query[i + 1].bytes;
                query[i].stopTX = query[i + 1].stopTX;
              }
            }
            queryIndex--;
          }
          break;
        default:
          break;
      }
      break;
    case I2C_CONFIG:
      delayTime = (argv[0] + (argv[1] << 7));

      if (argc > 1 && delayTime > 0) {
        i2cReadDelayTime = delayTime;
      }

      if (!isI2CEnabled) {
        enableI2CPins();
      }

      break;
    case SERVO_CONFIG:
      if (argc > 4) {
        // these vars are here for clarity, they'll optimized away by the compiler
        byte pin = argv[0];
        int minPulse = argv[1] + (argv[2] << 7);
        int maxPulse = argv[3] + (argv[4] << 7);

        if (IS_PIN_DIGITAL(pin)) {
          if (servoPinMap[pin] < MAX_SERVOS && servos[servoPinMap[pin]].attached()) {
            detachServo(pin);
          }
          attachServo(pin, minPulse, maxPulse);
          setPinModeCallback(pin, PIN_MODE_SERVO);
        }
      }
      break;
    case SAMPLING_INTERVAL:
      if (argc > 1) {
        samplingInterval = argv[0] + (argv[1] << 7);
        if (samplingInterval < MINIMUM_SAMPLING_INTERVAL) {
          samplingInterval = MINIMUM_SAMPLING_INTERVAL;
        }
      } else {
        //Firmata.sendString("Not enough data");
      }
      break;
    case EXTENDED_ANALOG:
      if (argc > 1) {
        int val = argv[1];
        if (argc > 2) val |= (argv[2] << 7);
        if (argc > 3) val |= (argv[3] << 14);
        analogWriteCallback(argv[0], val);
      }
      break;
    case SET_DIGITAL_PIN_VALUE:  // This is to set DO pin value
//      Serial3.printf("SET_DIGITAL\n");
//      Serial3.flush();

      if(argv[0] >= IO_Expander_DO_pin && argv[0] < IO_Expander_DO_pin+3) // this is io expander pin DI
      {
        IO_Expander_Write(argv[0] - IO_Expander_DO_pin, argv[1]);
      }
      setPinValueCallback(argv[0], argv[1]);
      break;  
    case CAPABILITY_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(CAPABILITY_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        if (IS_PIN_DIGITAL(pin)) {
          Firmata.write((byte)INPUT);
          Firmata.write(1);
          Firmata.write((byte)PIN_MODE_PULLUP);
          Firmata.write(1);
          Firmata.write((byte)OUTPUT);
          Firmata.write(1);
        }
        if (IS_PIN_ANALOG(pin)) {
          Firmata.write(PIN_MODE_ANALOG);
          Firmata.write(10); // 10 = 10-bit resolution
        }
        if (IS_PIN_PWM(pin)) {
          Firmata.write(PIN_MODE_PWM);
          Firmata.write(DEFAULT_PWM_RESOLUTION);
        }
        if (IS_PIN_DIGITAL(pin)) {
          Firmata.write(PIN_MODE_SERVO);
          Firmata.write(14);
        }
        if (IS_PIN_I2C(pin)) {
          Firmata.write(PIN_MODE_I2C);
          Firmata.write(1);  // TODO: could assign a number to map to SCL or SDA
        }
#ifdef FIRMATA_SERIAL_FEATURE
        serialFeature.handleCapability(pin);
#endif
        Firmata.write(127);
      }
      Firmata.write(END_SYSEX);
      break;
    case PIN_STATE_QUERY:
//      Serial3.printf("PIN_STATE_QUERY\n");
//      Serial3.flush();
      if (argc > 0) {
        byte pin = argv[0];
        Firmata.write(START_SYSEX);
        Firmata.write(PIN_STATE_RESPONSE);
        Firmata.write(pin);
        if (pin < TOTAL_PINS) {
          Firmata.write(Firmata.getPinMode(pin));
          Firmata.write((byte)Firmata.getPinState(pin) & 0x7F);
          if (Firmata.getPinState(pin) & 0xFF80) Firmata.write((byte)(Firmata.getPinState(pin) >> 7) & 0x7F);
          if (Firmata.getPinState(pin) & 0xC000) Firmata.write((byte)(Firmata.getPinState(pin) >> 14) & 0x7F);
        }
        Firmata.write(END_SYSEX);
      }
      break;
    case ANALOG_MAPPING_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(ANALOG_MAPPING_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        Firmata.write(IS_PIN_ANALOG(pin) ? PIN_TO_ANALOG(pin) : 127);
      }
      Firmata.write(END_SYSEX);
      break;

    case SERIAL_MESSAGE:
#ifdef FIRMATA_SERIAL_FEATURE
      serialFeature.handleSysex(command, argc, argv);
#endif
      break;
    case SYSEX_TMP102_REQUEST:  // 'd'
      readAndReportTMP102Data();
      break;
    case SYSEX_ACCEL_REQUEST:   // 'g'
      readAndReportAccelData();
      break;
    case SYSEX_COUNTER_REQUEST:
      readAndReportCounterData();
      break;
    case SYSEX_RTD_REQUEST:
      int rtd;
      rtd = argv[1];
      readAndReportRTD(rtd);
      break;       
    case SYSEX_LED_BLINK:
      {
        byte pin = argv[0];
        int delTime = argv[1] + argv[2] << 7;
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
        delay(delTime);
        digitalWrite(pin, LOW);
        delay(delTime);        
      }
      break;
    case SYSEX_485_REQUEST:
      //Serial1.print("RS485 command ");
      //Serial1.printf("RS485: %s\r\n",argv);

      //rs485.write(argv, sizeof(argv));
      count = merge_buffer(&argv[0],&buffer[0],argc);
      rs485.setTXmode();
      if(argv[0] > 0){
        rs485.write(&buffer[1], count-1);
      }
      rs485.setRXmode();
      //delayMicroseconds(100);//
      readRS485Data_return();
      break;
    case SYSEX_485_BAUDRATE_REQUEST:
      uint16_t rate,req;
      //rs485.setTXmode();
      //if(argv[0] > 0){
      //  rs485.write(&argv[1], argc-1);
      //}
      req = argv[0];
      rate = argv[1];
      if(req){
        switch(rate)
        {
          case 0:
            break;
          case 1:
            RS485_Baud_Rate = 9600;
            break;
          case 2:
            RS485_Baud_Rate = 19200;
            break;
          case 3:
            RS485_Baud_Rate = 38400;
            break;
          case 4:
            RS485_Baud_Rate = 57600;
            break;
          case 5:
            RS485_Baud_Rate = 115200;
            break;
        }
        Serial3.begin(RS485_Baud_Rate);
      }
      //Serial1.printf("RS485 baud rate=%d\r\n",RS485_Baud_Rate);
      readRS485BaudRate_return();
      break;
    case SYSEX_IOEXP_REQUEST:
      int io;
      io = argv[1];
      if(io)
        i2cRxData[0] = IO_Expander_Read_Reg(0x10);
      else
        i2cRxData[0] = IO_Expander_Read_Reg(0x11);
      //Serial1.printf("IOEXP=%d\r\n",i2cRxData[0]);
      
      //i2cRxData[0] = 0x5A;
      Firmata.sendSysex(SYSEX_IOEXP_REPLY, 1, i2cRxData);
      break;

    case SYSEX_CANBUS_REQUEST:
      
      if(argv[0] > 0){
        count = merge_buffer(&argv[0],&buffer[0],argc);
        //int i;
        //for(i=0;i<count;i++)
        //  Serial1.printf("IOEXP=%d\r\n",buffer[i]);
        SendCANBusData(&buffer[1], count-1);
      }
      ReadCANBusData_return();
      break;
    case SYSEX_FW_Version_REQUEST:
      uint32_t i,len;
      len = strlen(version);
      for (i=0;i<len;i++)
        i2cRxData[i] = version[i];
      Firmata.sendSysex(SYSEX_FW_Version_REPLY, len, i2cRxData);
      break;
    case SYSEX_SDCard_REQUEST:
      //Serial1.println(argv[0]);
      //Serial1.println(argv[1]);
      char func;
      if(argc == 0){
        result = 0;
        Firmata.sendSysex(SYSEX_SDCard_REPLY, 1, &result);  
        break;
      }
      func= char(argv[0]);
      //Serial1.println(argv[0]);
      //Serial1.println(argv[1]);
      //index = argv[1];
      result = 1;
      if (!SD.begin(true)) {
        //Serial1.println("SD card initialization failed!");
        result = 0;
        Firmata.sendSysex(SYSEX_SDCard_REPLY, 1, &result);  
        break;
      }
      index = argv[1];
      switch(func)
      {
        case 'c':  // create a file
          //Serial1.println(argv[1]);
          //if(argc > 13){
          //  result = 1;
          //  Firmata.sendSysex(SYSEX_SDCard_REPLY, 1, &result);  
          //  break;
          //}
          memset(&filename[0],0,256);
          for(int i=0;i<argc-1;i++)
            filename[i] = argv[1+i];
          //Serial1.println(i);
          //filename[i] = 0;
          //Serial1.println(argc);
          //Serial1.println(argv);
          //Serial1.println(filename);

          //if (!SD.begin(true)) {
          //  Serial1.println("SD card initialization failed!");
          //  result = 0;
          //  Firmata.sendSysex(SYSEX_SDCard_REPLY, 1, &result);  
          //  break;
          //}
          myFile = SD.open(filename, FILE_WRITE);
          // if the file opened okay, write to it:

          if (myFile) {
            //Serial1.print("Writing to ");
            //Serial1.print(filename);
            //myFile.println("testing 1, 2, 3.");
            // close the file:
            myFile.close();
            //Serial1.println("done.");
          } else {
          // if the file didn't open, print an error:
            //Serial1.print("error opening :");
            //Serial1.print(filename);
            result = 0;
            Firmata.sendSysex(SYSEX_SDCard_REPLY, 1, &result);  
            break;
          }
          result = 1;
          Firmata.sendSysex(SYSEX_SDCard_REPLY, 1, &result);  
          break;
        case 'l':  // list files 
          //Serial1.println(argv[1]);
          #if 0
            memset(&filename[0],0,256);
            for(int i=0;i<argc-1;i++)
              filename[i] = argv[1+i];
          #endif
          if(index == '0')
            sd_dir_index = 0;
          //Serial1.println(index);

          //if (!SD.begin(true)) {
          //  Serial1.println("SD card initialization failed!");
          //  result = 0;
          //  Firmata.sendSysex(SYSEX_SDCard_REPLY, 1, &result);  
          //  break;
          //}
          myDir = SD.open(filename);
          //s.clean();
          buff_index = 0;
          printDirectory(myDir, 0);
          result = 1;
          //Serial1.print("Size=");
          //Serial1.println(buff_index);
          int iindex;
          iindex = buff_index - sd_dir_index;
          if(iindex > 255)
            result = 255;
          else
            result = iindex;
          Firmata.sendSysex(SYSEX_SDCard_REPLY, result, &buffer[sd_dir_index]);  
          sd_dir_index = sd_dir_index + result -1;
          break;     
        case 'o':  // open a file
          memset(&filename[0],0,256);
          for(int i=0;i<argc-1;i++)
            filename[i] = argv[1+i];
          result = SD.exists(filename);
          if(strcmp(filename, "/")==0)
            result = 1;
          // if the file opened okay, write to it:
          if (result) {
            sd_write_index = 0;
            sd_read_index = 0;
            sd_dir_index = 0;
          } else {
            result = 0;
            Firmata.sendSysex(SYSEX_SDCard_REPLY, 1, &result);  
            break;
          }
          result = 1;
          Firmata.sendSysex(SYSEX_SDCard_REPLY, 1, &result);  
          break;
        case 'w':  // write files 
          int i;
          //Serial1.println(argv[1]);
          memset(&buffer[0],0,512);
          count = merge_buffer(&argv[2],&buffer[0],argc-2);
          //for(i=0;i<argc-2;i++)
          //  buffer[i] = argv[2+i];
          //Serial1.println((char *)&buffer[0]);
          myFile = SD.open(filename, FILE_WRITE);
          //s.clean();
          myFile.seek(sd_write_index);
          result = myFile.write(&buffer[0],count);
          myFile.close();
          sd_write_index = sd_write_index+ result;
          //Serial1.print("Size=");
          //Serial1.println(buff_index);
          Firmata.sendSysex(SYSEX_SDCard_REPLY, 1, &result);  
          break;     
        case 'r':  // read file 
          //Serial1.println(index);
          memset(&buffer[0],0,256);
          myFile = SD.open(filename);
          //Serial1.println(index);
          if(index == '0')
            sd_read_index = 0;
          //Serial1.println(sd_read_index);
          myFile.seek(sd_read_index);
          result = myFile.read(&buffer[0],255);
          //Serial1.print("result=");
          //Serial1.println(result);
          //Serial1.println((char*)&buffer[0]);

          sd_read_index = sd_read_index+ result;
          Firmata.sendSysex(SYSEX_SDCard_REPLY,result, &buffer[0]);  
          break;     
        case 'd':  // delete a file 
          memset(&t_filename[0],0,256);
          for(int i=0;i<argc-1;i++)
            t_filename[i] = argv[1+i];
          result = SD.remove(t_filename);
          //Serial1.printf("Delete File result=%d", result);
          Firmata.sendSysex(SYSEX_SDCard_REPLY, 1, &result);  
          break;
        case 'm':  // make a directory 
          memset(&t_filename[0],0,256);
          for(int i=0;i<argc-1;i++)
            t_filename[i] = argv[1+i];
          result = SD.mkdir(t_filename);
          //Serial1.printf("Delete File result=%d", result);
          Firmata.sendSysex(SYSEX_SDCard_REPLY, 1, &result);  
          break;
        case 'k':  // delete a directory
          memset(&t_filename[0],0,256);
          for(int i=0;i<argc-1;i++)
            t_filename[i] = argv[1+i];
          result = SD.rmdir(t_filename);
          //Serial1.printf("Delete File result=%d", result);
          Firmata.sendSysex(SYSEX_SDCard_REPLY, 1, &result);  
          break;
        case 'i':
          size = 0;
          if(result)
            size = SD.cardsize()/2048;
          i2cRxData[0] = size & 0x7F;
          i2cRxData[1] = (size >> 7) & 0x7F;
          i2cRxData[2] = (size >> 14) & 0x7F;
          i2cRxData[3] = (size >> 21) & 0x7F;
          Firmata.sendSysex(SYSEX_SDCard_REPLY, 4, i2cRxData);
          break;
      }
      break;
    case SYSEX_qspiFlash_REQUEST:
      //rs485.setTXmode();
      function = argv[0];
      index = argv[1];
      switch(function)
      {
        case 0:  // get id
          id = flash.getJEDECID();
          i2cRxData[0] = (id) & 0xFF;
          i2cRxData[1] = (id >> 8) & 0xFF;
          i2cRxData[2] = (id >> 16) & 0xFF;
          i2cRxData[3] = (id >> 24) & 0xFF;
          Firmata.sendSysex(SYSEX_qspiFlash_REPLY, 4, i2cRxData);
          break;
        case 1:  // erase all
          switch(index) {
            case 0: //Erase chip
              if(!flash.eraseChip()) {
                Serial1.println("Failed to erase chip!");
                result = 0;
              }
              else{
                flash.waitUntilReady();
                Serial1.println("Successfully erased chip!");
                result = 1;
              }
              Firmata.sendSysex(SYSEX_qspiFlash_REPLY, 1, &result);  
              break;             
            case 1: // Erase Sector
              count = merge_buffer(&argv[0],&buffer[0],argc);
              if(count == 6){
                number = (buffer[2] << 24) + (buffer[3] << 16) + (buffer[4] << 8) + buffer[5];
                Serial1.printf("Number=%d\r\n",number);
                result = flash.eraseSector(number);
              }
              else{
                result = 0;
              }
              //Serial1.printf("result=%d\r\n",result);
              Firmata.sendSysex(SYSEX_qspiFlash_REPLY, 1, &result);  
              break;
            case 2:
              count = merge_buffer(&argv[0],&buffer[0],argc);
              if(count == 6){
                number = (buffer[2] << 24) + (buffer[3] << 16) + (buffer[4] << 8) + buffer[5];
                Serial1.printf("Number=%d\r\n",number);
                result = flash.eraseBlock(number);
              }
              else{
                result = 0;
              }
              Firmata.sendSysex(SYSEX_qspiFlash_REPLY, 1, &result);  
              break;
          }
          break;

        case 2: //write
          //flash.writeEnable();

          //index = 0;
          //count = 0;
          //for(i=0;i<argc;i++){
          //  if(argv[i] >= 0xF0){
          //    buffer[index] = argv[i];
          //    i++;
          //    buffer[index] += argv[i];
          //  }
          //  else{
          //    buffer[index] = argv[i];
          //  }
          //  count++;
          //  index++;
          //}
          count = merge_buffer(&argv[0],&buffer[0],argc);

          address = (buffer[1] << 24) + (buffer[2] << 16) + (buffer[3] << 8) + buffer[4];
          if(count<=5){
            result = 0;
            //flash.writeDisable();
            Firmata.sendSysex(SYSEX_qspiFlash_REPLY, 1, &result);  
            break;
          }

          len = count-5;
          //if(len > 58) len = 58;
          //Serial1.printf("addr=%d\r\n",address);
          //Serial1.printf("len=%d\r\n",len);
          //for(i=0;i<argc;i++){
          //  Serial1.printf("%d %d\r\n ",i,buffer[i]);
          //}
          //Serial1.printf("Size=%d\r\n",size);
          if(len + address > spiFLASH_SIZE )
            len = spiFLASH_SIZE - address;
          //Serial1.printf("len=%d\r\n",len);
          size = flash.writeBuffer(address, &buffer[5], len);
          //Serial1.printf("size=%d\r\n",size);
          //for(i=0;i<size;i++){
          //  Serial1.printf("%d \r\n",buffer[i]);
          //}
          //size = flash.readBuffer(address, &buffer[0], len);
          //Serial1.printf("read size=%d\r\n",size);
          //for(i=0;i<size;i++){
          //  Serial1.printf("%d \r\n",buffer[i]);
          //}
          //Serial1.printf("Size=%d\r\n",size);
          result = (byte)size;
          //flash.writeDisable();
          Firmata.sendSysex(SYSEX_qspiFlash_REPLY, 1, &result);  
          break;     
        case 3: //read
          count = merge_buffer(&argv[0],&buffer[0],argc);
          address = (buffer[1] << 24) + (buffer[2] << 16) + (buffer[3] << 8) + buffer[4];
          len = (buffer[5] << 8) + buffer[6];
          if(len + address > spiFLASH_SIZE )
            len = spiFLASH_SIZE - address;
          //memset(&buffer[0],0,512);
          for(i=0;i<256;i++){
            buffer[i] = 0;
          //  Serial1.printf("%d %x \r\n",i,buffer[i]);
          }
          size = flash.readBuffer(address, &buffer[0], len);
          //Serial1.print("\r\n");
          //Serial1.printf("address=%d\r\n",address);
          //Serial1.printf("len=%d\r\n",len);
          //Serial1.printf("size=%d\r\n",size);
          //for(i=0;i<size;i++){
          //  Serial1.printf("%d %x \r\n",i,buffer[i]);
          //}
          Firmata.sendSysex(SYSEX_qspiFlash_REPLY, size, &buffer[0]);  
          break;        
      }
      break;

    case SYSEX_ResetCount_REQUEST:
      i2cRxData[0] = (power_reset_count) & 0x7F;
      i2cRxData[1] = (power_reset_count >> 7) & 0x7F;
      i2cRxData[2] = (power_reset_count >> 14) & 0x7F;
      i2cRxData[3] = (power_reset_count >> 21) & 0x7F;
      power_reset_count++;
      Firmata.sendSysex(SYSEX_ResetCount_REPLY, 4, i2cRxData);
      break;
      
  }
}


/*==============================================================================
 * SETUP()
 *============================================================================*/

void systemResetCallback()
{
  isResetting = true;

  // initialize a defalt state
  // TODO: option to load config from EEPROM instead of default

#ifdef FIRMATA_SERIAL_FEATURE
  serialFeature.reset();
#endif

  if (isI2CEnabled) {
    disableI2CPins();
  }

  for (byte i = 0; i < TOTAL_PORTS; i++) {
    reportPINs[i] = false;    // by default, reporting off
    portConfigInputs[i] = 0;  // until activated
    previousPINs[i] = 0;
    servoPinMap[i] = 255;
  }

#ifdef GEA_BOARD
  for(byte i = 0; i < GEA_TOTAL_DOUT_PINS; i++) {
    //Serial1.print("DO");
    setPinModeCallback(doutPinArray[i], OUTPUT);
  }

  for(byte i = 0; i < GEA_TOTAL_DIN_PINS; i++) {
    setPinModeCallback(dinPinArray[i], INPUT);
    //reportPINs[dinPinArray[i]] = true;
  }

  for(byte i = 0; i < GEA_TOTAL_AIN_PINS; i++) {
    setPinModeCallback(PIN_TO_ANALOG(ainPinArray[i]), PIN_MODE_ANALOG);
  }

  for(byte i = 0; i < GEA_TOTAL_AOUT_PINS; i++) {
    setPinModeCallback(PIN_TO_ANALOG(aoutPinArray[i]), PIN_MODE_ANALOG);
  }
  setPinModeCallback(MCU_IO_CH1_VOUT_EN, OUTPUT);
  initGEAIOs();
#else
  for (byte i = 0; i < TOTAL_PINS; i++) {
    // pins with analog capability default to analog input
    // otherwise, pins default to digital output
    if (IS_PIN_ANALOG(i)) {
      // turns off pullup, configures everything
      setPinModeCallback(i, PIN_MODE_ANALOG);
    } else if (IS_PIN_DIGITAL(i)) {
      // sets the output to 0, configures portConfigInputs
      setPinModeCallback(i, OUTPUT);
    }
    servoPinMap[i] = 255;
  }
#endif  
  // by default, do not report any analog inputs
  analogInputsToReport = 0;

  detachedServoCount = 0;
  servoCount = 0;

  /* send digital inputs to set the initial state on the host computer,
   * since once in the loop(), this firmware will only send on change */
  /*
  TODO: this can never execute, since no pins default to digital input
        but it will be needed when/if we support EEPROM stored config
  for (byte i=0; i < TOTAL_PORTS; i++) {
    outputPort(i, readPort(i, portConfigInputs[i]), true);
  }
  */
  isResetting = false;
}

void setup()
{

  Serial.begin(9600);
  Serial.println("USB Virtual Serial Port Initialized");
  ot2itEth.begin();
  ot2itEth.ip_stack_init((uint8_t *)MAC_ADDRESS);

  ot2it_tftp_example_init_server(); 
  //pinMode(27, OUTPUT);
  //digitalWrite(27, HIGH);
  uint32_t dd,i,numPages;
  uint32_t size;
  
  Serial1.begin(57600); // This is for debug port

  Serial1.print("\r\n");
  Serial1.println(version);
  // Flash test
  if (!flash.begin()) {
    Serial1.println("Error, failed to initialize flash chip!");
    //while (1) {
    //}
  }

  Serial1.print("Flash chip JEDEC ID: 0x");
  Serial1.println(flash.getJEDECID(), HEX);
  //Serial1.println("Chip Erase All");
  //dd = flash.read32(0);
  //Serial1.println(dd, HEX);
  numPages = flash.numPages();
  Serial1.printf("Flash Pages= %d\r\n",flash.numPages());
  Serial1.printf("Flash PageSize= %d\r\n",flash.pageSize());
  spiFLASH_SIZE = flash.size();
  Serial1.printf("Flash Size= %d\r\n",flash.size());

  //SD Card initialization
  if (!SD.begin(true)) {
    Serial1.println("SD card initialization failed!");
  }
  else{
    Serial1.println("SD card initialized!");
    Serial1.printf("SD card size = %d MB\r\n",SD.cardsize()/2048);
    //Serial1.printf("SD card size*block = %d\n",SD.cardsize()*SD_Block_Size);
  }
/*  
  myFile = SD.open("test4.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    Serial1.print("Writing to test4.txt...");
    myFile.println("testing 1, 2, 3.");
    // close the file:
    myFile.close();
    Serial1.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial1.println("error opening test4.txt");
  }

  // re-open the file for reading:
  myFile = SD.open("test4.txt");
  if (myFile) {
    Serial1.println("test4.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial1.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial1.println("error opening test4.txt");
  }
*/  
  /* //Erase Test
  if (!flash.eraseChip()) {
    Serial1.println("Failed to erase chip!");
  }
  flash.waitUntilReady();
  Serial1.println("Successfully erased chip!");

  for(i=0;i<247;i++){
    buffer[i] = i;
  }
  // Write Test
  flash.writeBuffer(0, &buffer[0], 247);
  for(i=0;i<256;i++){
    buffer[i] = 0;
  }
  //Read Test
  size = flash.readBuffer(0, &buffer[0], 255);
  for(i=0;i<size;i++){
    Serial1.printf("no = %d %x\r\n",i,buffer[i]);
  }
*/  
  //flash.writeEnable();
  //Serial1.print("Write enable\r\n");
  //uint8_t buff[10] = "1234";
  //size = flash.writeBuffer(0, &buff[0], 4);
  //dd = flash.read32(0);
  //Serial1.printf("size = %d\r\n",size);
  //Serial1.println(dd, HEX);

  //if (!flash.eraseChip()) {
  //  Serial1.println("Failed to erase chip!");
  //}
  //flash.waitUntilReady();
  //Serial1.println("Successfully erased chip!");


  Firmata.setFirmwareVersion(FIRMATA_FIRMWARE_MAJOR_VERSION, FIRMATA_FIRMWARE_MINOR_VERSION);
  
  Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
  Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
  //Firmata.attach(DIGITAL_MESSAGE, setPinValueCallback);
  Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
  Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
  Firmata.attach(SET_PIN_MODE, setPinModeCallback);
  Firmata.attach(SET_DIGITAL_PIN_VALUE, setPinValueCallback);
  Firmata.attach(START_SYSEX, sysexCallback);
  Firmata.attach(SYSTEM_RESET, systemResetCallback);
  
  // to use a port other than Serial, such as Serial1 on an Arduino Leonardo or Mega,
  // Call begin(baud) on the alternate serial port and pass it to Firmata to begin like this:
  //Serial3.begin(57600); // This is for RS485

  // Serial3.begin(57600);
  // Firmata.begin(Serial3);
  // However do not do this if you are using SERIAL_MESSAGE

  Firmata.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for ATmega32u4-based boards and Arduino 101
  }
  systemResetCallback();  // reset to default config
}
/*==============================================================================
 * LOOP()
 *============================================================================*/

void loop()
{
  byte pin, analogPin;
  int val;
  static int enableState_G, enableState_R ;
  int count = 0;
  //char test[] = "123456789\r\n";
  //static byte show_version = 0;

  ot2itEth.get_link_sts();
  ot2itEth.receive();
  /*Initialize IP stack interface with MAC Address*/
  
  /*
  while(1){
    if (Serial1.available()) {
      int inByte = Serial1.read();
      Serial1.write(inByte);
    }
  }
  */
   /* DIGITALREAD - as fast as possible, check for changes and output them to the
    *FTDI buffer using Serial.print()  */
  //if(!show_version){
  //  Serial1.print("MCU Version 1.00\r\n");
  //  show_version = 1;
  //}
  /*
  // Test for rs485
#if Use_ArduinoRS485      
  rs485.beginTransmission();
  rs485.print(test);
  rs485.endTransmission();
#else
  rs485.setTXmode();
  rs485.write(test, sizeof(test));
#endif
  */
  // test for I2C/ io expander
  //IO_Expander_Read(0, 1);
  //IO_Expander_Write(0, 0xFF);
  checkDigitalInputs();

  val = digitalRead(MCU_KEY_1);
  if (val == LOW) {
    enableState_G = !enableState_G;
//    digitalWrite(MCU_LED_G, enableState_G);
  }

  val = digitalRead(MCU_KEY_2);
  if (val == LOW) {
    enableState_R = !enableState_R;
//    digitalWrite(MCU_LED_R, enableState_R);
  }
  readRS485Data();
  //SendCANBusData();
  ReadCANBusData();
  
// To make interrupt pin go back to high to avoid undetected bouncing
  if(!digitalRead(DI7_Pulse_Pin)){
    noInterrupts();
    IO_Expander_Write_Reg(0x19,0xFF);
    interrupts();
  }
  /*
  //readDebugData();
	if (!p.isEmpty())				// Only if q is not empty
	{
	  char snd;
    int i=0,count;
    count = q.getCount();
    for(i=0;i<count;i++){
	   q.pop(&snd);		// While characters are available in q
	   Serial1.write(snd);
     if(snd == 'v' || snd == 'V')
       Serial1.print(version);
	  }
	}
  */
  while (Serial1.available())					// Ensure Serial is available
	{
	  char rcv = (char) Serial1.read();	// Read char from Serial
    Serial1.write(rcv);
    if(rcv == 'v' || rcv == 'V')
      Serial1.print(version);
  }

  /*
  val = digitalead(MCU_KEY_1);
  if (val == HIGH) {
    digitalWrite(MCU_LED_G, LOW);
  }else{
    digitalWrite(MCU_LED_G, HIGH);
  }

  val = digitalRead(MCU_KEY_2);
  if (val == HIGH) {
    digitalWrite(MCU_LED_R, LOW);
  }else{
    digitalWrite(MCU_LED_R, HIGH);
  } */

  /* STREAMREAD - processing incoming messagse as soon as possible, while still
   * checking digital inputs.  */
  while (Firmata.available())
    Firmata.processInput();

  // TODO - ensure that Stream buffer doesn't go over 60 bytes

  currentMillis = millis();
  if (currentMillis - previousMillis > samplingInterval) {
    previousMillis += samplingInterval;
    /* ANALOGREAD - do all analogReads() at the configured sampling interval */
    for (pin = 0; pin < TOTAL_PINS; pin++) {
      /* if (IS_PIN_ANALOG(pin) && Firmata.getPinMode(pin) == PIN_MODE_ANALOG) { */
      if (IS_PIN_ANALOG(pin)) {
        analogPin = PIN_TO_ANALOG(pin);
        //digitalWrite(MCU_LED_R, HIGH);
        if (analogInputsToReport & (1 << analogPin)) 
        {
          //Firmata.sendAnalog(analogPin, analogRead(pin)); //analogRead(analogPin));
          //Firmata.sendAnalog(pin, analogRead(pin));
          //Serial1.printf("analogRead pin= %d, value= %d\r\n",analogPin,analogRead(ANALOG_TO_PIN(analogPin)));
          Firmata.sendAnalog(analogPin, analogRead(ANALOG_TO_PIN(analogPin)));
        }
      }
    }
    // report i2c data for all device with read continuous mode enabled
    if (queryIndex > -1) {
      for (byte i = 0; i < queryIndex + 1; i++) {
        readAndReportData(query[i].addr, query[i].reg, query[i].bytes, query[i].stopTX);
      }
    }
  }
  /*
  if(rs485.getMode())
    rs485.setRXmode();
	if (rs485.available())					// Ensure Serial is available
	{
		char rcv = (char) rs485.read();	// Read char from Serial

		if (rcv != -1)						// If no character received: -1 (do not push)
		{
			q.push(&rcv);					// Push char to cppQueue
		}
	}
  */
#ifdef FIRMATA_SERIAL_FEATURE
  serialFeature.update();
#endif
}

//------------------------------------------------------------------------------
void IO_Exapnader_Init()
{
  byte data;

  digitalWrite(IO_Expander_NRST_pin, HIGH);

  // Set Port 0 pin directions
  IO_Expander_Write_Reg(0x0E, 0x80); // set Register B as output
  IO_Expander_Write_Reg(0x0F, 0xFF); // set Register A as input

  IO_Expander_Write_Reg(0x10, 0x00);

  //Set interrupt for DI 7 pulse (IO6) , falling
  // 0x13 RegInterruptMaskA
  IO_Expander_Write_Reg(0x13, 0xBF); // 1011 1111
  // 0x16 RegSenseHighA Sense register for I/O[7:4]
  IO_Expander_Write_Reg(0x16, 0x20); // xx10xxxx
 }
void IO_Expander_Write_Reg(byte reg, byte value)
{
  //Serial1.printf("pin= %d value=%d\r\n",pin,value);
  Wire.begin(); //Join I2C Bus   
  Wire.setClock(400000);
  Wire.beginTransmission(IO_Ex_Slave_Addr);
  Wire.write(reg);  //
  Wire.write(value);  // Register B 8 bits as output
  Wire.endTransmission();
  ////delayMicroseconds(70);
}

byte IO_Expander_Read_Reg(byte reg)
{
byte data,count;
 
  Wire.begin(); //Join I2C Bus   
  Wire.setClock(400000);
  Wire.beginTransmission(IO_Ex_Slave_Addr);
  Wire.write(reg);  // RegDirA
  Wire.endTransmission();
  //Wire.beginTransmission(IO_Ex_Slave_Addr);
  count = Wire.requestFrom(IO_Ex_Slave_Addr, 1, 1);
  data = Wire.read();  // Register B 8 bits as output
  //Serial1.printf("Read data %d\r\n", data);
  //Wire.endTransmission();
  ////delayMicroseconds(70);
  return data;
}

void IO_Expander_Write(byte pin, byte value)
{
  static byte IO_Expander_PortB_Value = 0x00;
  // Set Port 0 pin directions
  byte pin_index;
  //Serial1.printf("pin= %d value=%d\r\n",pin,value);
  switch (pin)
  {
    case 0: pin_index = 1; break;
    case 1: pin_index = 2; break;
    case 2: pin_index = 5; break;
    default: return;
  }
  if(value > 0) value = 1;
  //if(value > 0) 
  //  value = 0;
  //else
  //  value = 1;
  IO_Expander_PortB_Value &= (~(1 << pin_index));
  IO_Expander_PortB_Value |= (value << pin_index);
  //IO_Expander_PortB_Value = value;
  //Serial1.printf("pin index = %d, Write Value=%d\r\n",pin_index,IO_Expander_PortB_Value);
  IO_Expander_Write_Reg(0x10, IO_Expander_PortB_Value);
}

byte IO_Expander_Read(byte bank, byte pin)
{
byte data,count;
byte pin_index;
 
  switch (pin)
  {
    case 0: pin_index = 1; break;
    case 1: pin_index = 2; break;
    case 2: pin_index = 6; break;
  }
  if(bank ==0)
    data = IO_Expander_Read_Reg(0x11);
  else
    data = IO_Expander_Read_Reg(0x10);

  //Serial1.printf("pin=%d, index = %d, Read data %d\r\n", pin, pin_index, data);
  data = data & (1<<pin_index);

  if(data)
    return 1;
  else
    return 0;
}

uint32_t merge_buffer(byte *input,byte *output,byte argc)
{
  uint32_t i,index,count;

  index = 0;
  count = 0;
  
  for(i=0;i<argc;i++){
//    if(input[i] >= 0xF0){
    if(input[i] == 0xF6){
      output[index] = input[i];
      i++;
      output[index] += input[i];
    }
    else{
      output[index] = input[i];
    }
    count++;
    index++;
  }
  return count;
}

void printDirectory(File2 dir, int numTabs) {
  int i,size;
  char *P;
  while (true) {

    File2 entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      //Serial1.print('\t');
      if(buff_index < BUFF_SIZE)
        buffer[buff_index++] = '\t';
    }
    //Serial1.print(entry.name());
    size = strlen(entry.name());
    P = entry.name();
    for(i=0;i<size;i++){
      if(buff_index < BUFF_SIZE){
        buffer[buff_index++] = P[i];
      }
    }
    if (entry.isDirectory()) {
      //Serial1.println("/");
      if(buff_index < BUFF_SIZE)
        buffer[buff_index++] = '/';
      if(buff_index < BUFF_SIZE)
        buffer[buff_index++] = '\n';
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      //Serial1.print("\t\t");
      if(buff_index < BUFF_SIZE)
        buffer[buff_index++] = '\t';
      if(buff_index < BUFF_SIZE)
        buffer[buff_index++] = '\t';
      //Serial1.println(entry.size(), DEC);
      sprintf((char*)&buffer[buff_index],"%d\n",entry.size());
      int len;
      len = strlen((char*)&buffer[buff_index]);
      buff_index = buff_index + len;
      //buffer[buff_index++] = '\n';
    }
    entry.close();
  }
}