/*
  OT2ITEthernetStream.h

  Copyright (C) 2017 Marc Josef Pees. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.

  Last updated July 10th, 2017
 */

#ifndef OT2ITEthernetStream_H
#define OT2ITEthernetStream_H

#include <inttypes.h>
#include <Stream.h>
#include <Ethernet.h>
#include "Firmata.h"

//#define SERIAL_DEBUG
//#include "firmataDebug.h"
#include "lwip_raw_api.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "lwipopts.h"

#define HOST_CONNECTION_DISCONNECTED 0
#define HOST_CONNECTION_CONNECTED    1

struct ot2it_buf {
    uint8_t send_buf[128];
    uint8_t recv_buf[128];
    uint8_t send_ptr;
    uint8_t recv_ptr;
};

struct ot2it_buf obuf;

typedef void (*host_conn_cb_fn)(byte state);
static host_conn_cb_fn conn_cb_fn = NULL;

struct http_state {
	char *file;
	u32_t left;
	u8_t  retries;
};

using namespace firmata;

class OT2ITEthernetStream : public Stream
{
  public:
    OT2ITEthernetStream(IPAddress localip, uint16_t port);
    int available();
    int read();
    int peek();
    void flush();
    size_t write(uint8_t);
    void maintain(IPAddress localip, int port);
    void attach(host_conn_cb_fn fn);

  private:
    EthernetClient client;
    IPAddress localip;
    uint16_t port;
    bool connected;
    bool maintain();
    void stop();
    struct tcp_pcb *pcb;
    
  protected:
    EthernetServer server = EthernetServer(3030);
    bool listening = false;
    int listen_port = 3030;
    bool connect_client();
};

/**
 * \brief Close connection.
 *
 * \param pcb Pointer to a TCP connection structure.
 * \param hs Pointer to structure representing the HTTP state.
 */
static void http_close_conn(struct tcp_pcb *pcb)
{
	tcp_arg(pcb, NULL);
	tcp_sent(pcb, NULL);
	tcp_recv(pcb, NULL);
	tcp_close(pcb);
}

/*
 * OT2ITEthernetStream.cpp
 * Copied here as a hack to linker issues with 3rd party board packages that don't properly
 * implement the Arduino network APIs.
 */
OT2ITEthernetStream::OT2ITEthernetStream(IPAddress localip, uint16_t port)
  : localip(localip),
    port(port),
    connected(false)
{
}

bool OT2ITEthernetStream::connect_client()
{
    if ( connected )
    {
      if ( client && client.connected() ) return true;
      stop();
    }

    EthernetClient newClient = server.available();
    if ( !newClient ) return false;
    client = newClient;
    connected = true;
    
    return true;
}

int
OT2ITEthernetStream::available()
{
  return maintain() ? client.available() : 0;
}

int
OT2ITEthernetStream::read()
{
    if(obuf.recv_ptr > 0) {
        return obuf.recv_buf[--obuf.recv_ptr];
    }else{
      return -1;
    }
}

int
OT2ITEthernetStream::peek()
{
  return maintain() ? client.peek() : -1;
}

void OT2ITEthernetStream::flush()
{
  if (maintain())
    client.flush();
}

size_t
OT2ITEthernetStream::write(uint8_t c)
{
    static int len =0;
    err_t err = ERR_OK;

    switch(c) {
        case START_SYSEX:
            len=0;
            obuf.send_ptr = 0;
            obuf.send_buf[obuf.send_ptr++];
            len++;
            break;
        case END_SYSEX:
            obuf.send_buf[obuf.send_ptr++];
            len++;
            if((len == obuf.send_ptr) && (obuf.send_ptr > 3)) {
                do {
		            /* Use copy flag to avoid using flash as a DMA source (forbidden). */
		            err = tcp_write(pcb, obuf.send_buf, obuf.send_ptr, TCP_WRITE_FLAG_COPY);
	            } while (err == ERR_MEM);
            }
            break;
        default:
            obuf.send_buf[obuf.send_ptr++];
            break;
    }
    return err;
}

void
OT2ITEthernetStream::maintain(IPAddress localip, int port)
{
  // ensure the local IP is updated in the case that it is changed by the DHCP server
  if (this->localip != localip) {
    this->localip = localip;
    if (connected)
      stop();
  }
  listen_port = port;
  maintain();
}

void
OT2ITEthernetStream::stop()
{
  if(pcb != NULL)
  {
    http_close_conn(pcb);
  }
  connected = false;
}

/**
 * \brief Poll for HTTP data.
 *
 * \param arg Pointer to structure representing the HTTP state.
 * \param pcb Pointer to a TCP connection structure.
 *
 * \return ERR_OK on success, ERR_ABRT otherwise.
 */
static err_t http_poll(void *arg, struct tcp_pcb *pcb)
{
	struct http_state *hs;

	hs = (struct http_state *)arg;

	if (hs == NULL) {
		tcp_abort(pcb);
		return ERR_ABRT;
	} else {
		if (hs->file == 0) {
			tcp_abort(pcb);
			return ERR_ABRT;
		}

		++hs->retries;
		if (hs->retries == 4) {
			tcp_abort(pcb);
			return ERR_ABRT;
		}

		//http_send_data(pcb, hs);
	}

	return ERR_OK;
}

/**
 * \brief Core server receive function. Handle the request and process it.
 *
 * \param arg Pointer to structure representing the HTTP state.
 * \param pcb Pointer to a TCP connection structure.
 * \param p Incoming request.
 * \param err Connection status.
 *
 * \return ERR_OK.
 */
static err_t http_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
  if(p->len < 128) {
    memcpy(obuf.recv_buf, p->payload, p->len);
    obuf.recv_ptr = p->len;
  }   
	return ERR_OK;
}

static err_t http_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
	struct http_state *hs;

	LWIP_UNUSED_ARG(arg);
	LWIP_UNUSED_ARG(err);

	tcp_setprio(pcb, TCP_PRIO_MIN);

	/* Allocate memory for the structure that holds the state of the
	connection. */
	hs = (struct http_state *)mem_malloc(sizeof(struct http_state));

	if (hs == NULL) {
		/* http_accept: Out of memory */
		return ERR_MEM;
	}

	/* Initialize the structure. */
	hs->file    = NULL;
	hs->left    = 0;
	hs->retries = 0;

	/* Tell TCP that this is the structure we wish to be passed for our
	callbacks. */
	tcp_arg(pcb, hs);

	/* Tell TCP that we wish to be informed of incoming data by a call
	to the http_recv() function. */
	tcp_recv(pcb, http_recv);

	tcp_poll(pcb, http_poll, 4);
	return ERR_OK;
}

bool
OT2ITEthernetStream::maintain()
{
  //if (connect_client()) return true;
  
  //stop();
  
  if(!listening)
  {
    pcb = tcp_new();
    tcp_bind(pcb, IP_ADDR_ANY, listen_port);
    Serial.println("listening");
    pcb = tcp_listen(pcb);
    if (pcb != NULL) {
      tcp_accept(pcb, http_accept);
    }

    listening = true;
  }
  return false;
}

void OT2ITEthernetStream::attach(host_conn_cb_fn fn)
{
    conn_cb_fn = fn;
}
#endif /* OT2ITEthernetStream_H */