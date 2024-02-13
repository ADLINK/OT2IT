#include <OT2ITEth.h>

#define MAC_ADDRESS 0x11

OT2ITEth ot2itEth;

void setup() {
    Serial.begin(9600);
    ot2itEth.begin();  
}

void loop() {
  /*Get physical link status */
  ot2itEth.get_link_sts();
  /*Initialize IP stack interface with MAC Address*/
  ot2itEth.ip_stack_init((uint8_t *)MAC_ADDRESS);
   
   while(true){
    /*
     *Handle web server events, Wait to receive packets,
     *process the ethernet packets 
     *Print IP information
     */
    ot2itEth.receive();
   }
}
