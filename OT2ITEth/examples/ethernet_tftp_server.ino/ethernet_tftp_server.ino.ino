#include <OT2ITEth.h>
//#include <SD.h>

#define MAC_ADDRESS 0x11

uint8_t mac[6] = {0x00, 0xBE, 0x43, 0x75, 0x32, 0xCC};

OT2ITEth ot2itEth;

#ifdef __cplusplus
extern "C" {
#endif

#include <ping.h>

void tftp_example_init_server(void);

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
#define LWIP_TFTP_EXAMPLE_CLIENT_FILENAME "test.txt"
#endif

/* Define this to a server IP string */
#ifndef LWIP_TFTP_EXAMPLE_CLIENT_REMOTEIP
#define LWIP_TFTP_EXAMPLE_CLIENT_REMOTEIP "192.168.1.66"
#endif

static char full_filename[256];
#define DUMMY_FILE_ACCESS
#ifdef DUMMY_FILE_ACCESS
static char file_rd_data[512] = "ADLINK Technologies OT2IT Text data"; 
static char file_wr_data[4096];
static int data_cnt;
#endif

void *tftp_open_file(const char* fname, u8_t is_write)
{
#ifdef DUMMY_FILE_ACCESS
  data_cnt = 0;
  return (void *)0x1234;
#else
  snprintf(full_filename, sizeof(full_filename), "%s%s", LWIP_TFTP_EXAMPLE_BASE_DIR, fname);
  full_filename[sizeof(full_filename)-1] = 0;

  if (is_write) {
    return (void*)fopen(full_filename, "wb");
  } else {
    return (void*)fopen(full_filename, "rb");
  }
#endif  
}

void*
tftp_open(const char* fname, const char* mode, u8_t is_write)
{
  Serial.println("tftp_open called");
  return tftp_open_file(fname, is_write);
}

void tftp_close(void* handle)
{
  Serial.println("tftp_close called");
#ifndef DUMMY_FILE_ACCESS  
  fclose((FILE*)handle);
#endif  
}

int tftp_read(void* handle, void* buf, int bytes)
{
#ifdef DUMMY_FILE_ACCESS
  data_cnt += bytes;
  if(data_cnt > 1024)
    return 0;
  if(bytes <= 1024) {
    memcpy(buf, file_rd_data, bytes);
  }
  return bytes;
#else  
  int ret = fread(buf, 1, bytes, (FILE*)handle);
  Serial.println("tftp_read called");
  if (ret <= 0) {
    return -1;
  }
  return ret;
#endif  
}

int tftp_write(void* handle, struct pbuf* p)
{
#ifdef DUMMY_FILE_ACCESS
  Serial.println("tftp_write called");
  while(p != NULL) {
    if(p->len <= 4096) {
      memcpy(file_wr_data, p->payload, p->len);
    }else
      return 0;
    p = p->next;
  }
#else
  Serial.println("tftp_write called");
  while (p != NULL) {
    if (fwrite(p->payload, 1, p->len, (FILE*)handle) != (size_t)p->len) {
      return -1;
    }
    p = p->next;
  }

  return 0;
#endif  
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

const struct tftp_context tftp = {
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

int ot2it_tftp_example_init_client(void)
{
  void *f;
  err_t err;
  ip_addr_t srv;
  int ret = ipaddr_aton(LWIP_TFTP_EXAMPLE_CLIENT_REMOTEIP, &srv);
  Serial.println("ipaddr_aton failed");

  err = tftp_init_client(&tftp);
  if(err != ERR_OK) {
    Serial.println("tftp_init_client failed");
    return -1;
  }

  f = tftp_open_file(LWIP_TFTP_EXAMPLE_CLIENT_FILENAME, 1);
  if(f == NULL) {
    Serial.println("failed to create file");
    return -1;
  }
  err = tftp_get(f, &srv, TFTP_PORT, LWIP_TFTP_EXAMPLE_CLIENT_FILENAME, TFTP_MODE_OCTET);
  if(err == ERR_OK){
    Serial.println("tftp get success");
  }else{
    Serial.println("tftp get failed");
    return -1;
  }
  return 0;
  //LWIP_ASSERT("tftp_get failed", err == ERR_OK);
}

#ifdef __cplusplus
}
#endif

void setup() {
    Serial.begin(9600);
    ot2itEth.begin(); 
}

void ot2it_print_ipaddress(void);
int ping_res_fn(int ret) {
  if(ret < 0) {
    Serial.println("Ping: Timedout");
  }else{
    Serial.print(ret);
    Serial.println(" ms ping response");
  }
}
void loop() {
  static int ret = -1;
  static int init = 0;
  static unsigned long pingtime = 0;
  // Serial.print("Initializing SD card...");

  // if (!SD.begin(true)) {
  //   Serial.println("initialization failed.");
  // }else{
  //   Serial.println("initialization done.");
  // }

  if(init == 0) {
    //ot2itEth.force_10mbps();
    /*Get physical link status */
    ot2itEth.get_link_sts();

    /*Initialize IP stack interface with MAC Address*/
    ot2itEth.ip_stack_init((uint8_t *)mac);
    ping_init();
    ot2it_tftp_example_init_server();
    init = 1;
  }
  //while(true)
  {
    /*
     *Handle web server events, Wait to receive packets,
     *process the ethernet packets 
     *Print IP information
     */
    ot2itEth.receive();
    if(ot2itEth.maintain()) {
      //Serial.print("pingtime: ");
      //Serial.println(pingtime);
      if(pingtime == 0)
      {
        Serial.println("Pinging 192.168.1.66");
        ping_send_now("192.168.1.66", ping_res_fn);
        pingtime = 10000000;
      }else{
        pingtime--;
      }
    }
  }
}
