#ifndef OT2IT_ETHERNET_H
#define OT2IT_ETHERNET_H

#include <stdint.h>
#include "variant.h"
#include "Arduino.h"
#include <conf_eth.h>
#include <ethernet_phy.h>
#include <ethernet_phy_main.h>
#include <driver_init.h>
#include "peripheral_clk_config.h"
#include "lwip_macif_config.h"
#include "lwip_raw_api.h"
#include "lwip/tcpip.h"
#include "lwip/sockets.h"
#include "hri_gmac.h"
#include "eth_ipstack_main.h"
#include "netif/etharp.h"
#include "lwip/memp.h"
#include "lwip/dhcp.h"
#include "sam.h"
#include "lwip/timers.h"
#include <IPAddress.h>

/* global variables */
extern struct ethernet_phy_descriptor ETHERNET_PHY_0_desc;
extern struct mac_async_descriptor COMMUNICATION_IO;
extern struct netif TCPIP_STACK_INTERFACE_0_desc;
extern bool link_up;
extern uint8_t speed;
//extern char buffer[1024];

/* Saved total time in ms since timer was enabled */
extern volatile uint32_t systick_timems;
volatile static bool recv_flag = false;

#ifdef __cplusplus
extern "C" {
#endif
extern uint32_t sys_now(void);
void systick_enable(void);
#ifdef __cplusplus
}
#endif

class OT2ITEth
{
public:
   void begin();
   void begin(uint8_t *hwaddr);
   void get_link_sts();
   void ip_stack_init(uint8_t *hwaddr);
   void receive();
   IPAddress localIP(void);
   void setIPAddress(IPAddress ipAddr);
   void setGatewayIp(IPAddress ipAddr);
   void setSubnetMask(IPAddress ipAddr);
   int maintain(void);
};
#endif