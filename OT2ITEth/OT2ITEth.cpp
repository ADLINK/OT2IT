#include "OT2ITEth.h"


bool link_up = false;
bool dhcp_up = false;
uint8_t speed;
char buffer[1024];

/* This is the data for the socket web page. */
const static char socket_webpage[] = "<html> \
<head><title>Basic webpage</title></head> \
<body> \
Welcome to your basic webpage using Socket API. \
</body> \
</html>";
const static char http_html_hdr[] = "HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n";

#define OT2IT_ASSERT(message, assertion) \
  do                                     \
  {                                      \
    if (!(assertion))                    \
      Serial.println(message);           \
  } while (0)

void ot2it_print_ipaddress(void)
{
  static char tmp_buff[16];
  Serial.print("IP_ADDR    : ");
  Serial.println(ipaddr_ntoa_r((const ip_addr_t *)&(TCPIP_STACK_INTERFACE_0_desc.ip_addr), tmp_buff, 16));
  Serial.print("NET_MASK   : ");
  Serial.println(ipaddr_ntoa_r((const ip_addr_t *)&(TCPIP_STACK_INTERFACE_0_desc.netmask), tmp_buff, 16));
  Serial.print("GATEWAY_IP : ");
  Serial.println(ipaddr_ntoa_r((const ip_addr_t *)&(TCPIP_STACK_INTERFACE_0_desc.gw), tmp_buff, 16));
}

u32_t sys_now(void)
{
  return systick_timems;
}

void SysTick_Handler(void)
{
  systick_timems++;
}

void systick_enable(void)
{
  systick_timems = 0;
  SysTick_Config((CONF_CPU_FREQUENCY) / 1000);
}

void mac_receive_cb(struct mac_async_descriptor *desc)
{
  recv_flag = true;
}

void link_status_print(void)
{
  Serial.print("Eth: ");
  if (link_up)
  {
    Serial.print("UP. ");
    switch (speed)
    {
    case 1:
      Serial.println("10Mbps Half Dx");
      led_10_100_off();
      break;
    case 2:
      Serial.println("10Mbps Full Dx");
      led_10_100_off();
      break;
    case 3:
      Serial.println("100Mbps Half Dx");
      led_10_100_on();
      break;
    case 4:
      Serial.println("100Mbps Full Dx");
      led_10_100_on();
      break;
    default:
      break;
    }
  }
  else
  {
    Serial.println("Down");
  }
}

void OT2ITEth::begin()
{
  system_init();
  Serial.println("Serial1 Setup");
  delay(2000);
  ethernet_phys_init();
  systick_enable();
  mac_async_register_callback(&COMMUNICATION_IO, MAC_ASYNC_RECEIVE_CB, (FUNC_PTR)mac_receive_cb);
  eth_ipstack_init();
  Serial.println("Ethernet IP Stack Initialized");
}

void OT2ITEth::begin(uint8_t *hwaddr)
{
  system_init();
  Serial.println("Serial1 Setup");
  delay(2000);
  ethernet_phys_init();
  systick_enable();
  mac_async_register_callback(&COMMUNICATION_IO, MAC_ASYNC_RECEIVE_CB, (FUNC_PTR)mac_receive_cb);
  eth_ipstack_init();
  Serial.println("Ethernet IP Stack Initialized");
  ip_stack_init(hwaddr);
}

int32_t OT2ITEth::force_10mbps()
{
  int32_t ret;
  //mac_async_deinit(&COMMUNICATION_IO);
  //ot2it_speed = 0;
  ret = ethernet_phy_force_10mbps(&ETHERNET_PHY_0_desc);
  if(ret != ERR_NONE) {
    Serial.println("Ethernet: Force 10 MBPS Failed");
  }else{
    Serial.println("Ethernet: Force 10 MBPS Done");
  }
  //mac_async_init(&COMMUNICATION_IO, GMAC);
  return ret;
}

int32_t OT2ITEth::force_100mbps()
{
  int32_t ret;
  //mac_async_deinit(&COMMUNICATION_IO);
  //ot2it_speed = 1;  
  ret = ethernet_phy_force_100mbps(&ETHERNET_PHY_0_desc);
  if(ret != ERR_NONE) {
    Serial.println("Ethernet: Force 100 MBPS Failed");
  }else{
    Serial.println("Ethernet: Force 100 MBPS Done");
  }
  //mac_async_init(&COMMUNICATION_IO, GMAC);  
  return ret;
}

void OT2ITEth::get_link_sts()
{
  int ret;
  static int lup;
  do
  {
    ret = ethernet_phy_get_link_status(&ETHERNET_PHY_0_desc, &link_up, &speed);
    if (link_up)
    {
      break;
    }
    delay(200);
  } while (true);

  if(lup != link_up) {
    if(link_up) {
      Serial.println("Ethernet link up");
      link_status_print();
    }
    lup = link_up;
  }

  //if (!(link_up && TCPIP_STACK_INTERFACE_0_desc.ip_addr.addr)) {
    receive();
  //}
}

void OT2ITEth::ip_stack_init(uint8_t *hwaddr)
{
  TCPIP_STACK_INTERFACE_0_init(hwaddr);
#if CONF_TCPIP_STACK_INTERFACE_0_DHCP
  /* DHCP mode. */
  if (ERR_OK != dhcp_start(&TCPIP_STACK_INTERFACE_0_desc))
  {
    LWIP_ASSERT("ERR_OK != dhcp_start", 0);
  }
#else
  netif_set_up(&TCPIP_STACK_INTERFACE_0_desc); // enable this for Static IP
#endif

  //ot2it_print_ipaddress();
  /*Handles web server events*/
  //lwip_raw_api_init();
}

void OT2ITEth::receive()
{
  if (recv_flag)
  {
    recv_flag = false;
    ethernetif_mac_input(&TCPIP_STACK_INTERFACE_0_desc);
  }
  /* LWIP timers - ARP, DHCP, TCP, etc. */
  sys_check_timeouts();

  /* Print IP address info */
  if (link_up && TCPIP_STACK_INTERFACE_0_desc.ip_addr.addr)
  {
    //link_up = false;
    if(dhcp_up == false) {
      ot2it_print_ipaddress();
      dhcp_up = true;
    }
  }else{
    dhcp_up = false;
  }
}

IPAddress OT2ITEth::localIP(void)
{
  return IPAddress(TCPIP_STACK_INTERFACE_0_desc.ip_addr.addr);
}

void OT2ITEth::setIPAddress(IPAddress ipAddr)
{
  TCPIP_STACK_INTERFACE_0_desc.ip_addr.addr = ipAddr;
}

void OT2ITEth::setGatewayIp(IPAddress ipAddr)
{
  TCPIP_STACK_INTERFACE_0_desc.gw.addr = ipAddr;
}

void OT2ITEth::setSubnetMask(IPAddress ipAddr)
{
  TCPIP_STACK_INTERFACE_0_desc.netmask.addr = ipAddr;
}

int OT2ITEth::maintain(void)
{
  return (link_up && dhcp_up);
}
