/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
/* Includes ------------------------------------------------------------------*/
#include <string.h>      /* memcpy() */
#include "udpServer.h"
#include "config.h"     /* MulticastReceived() */

#include "lwip/opt.h"

#if LWIP_NETCONN

#include "lwip/api.h"
#include "lwip/sys.h"
#include "lwip.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define UDPECHO_THREAD_PRIO     (osPriorityNormal)
#define UDP_PROTOCOL_THREAD_PRIO (osPriorityNormal)
#define UDPSEND_THREAD_PRIO     (osPriorityNormal)

#define UDP_SERVER_PORT         55555

#define BUFFER_LENGTH           200
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t udpResponsePending;
GenericMessageType CurrentUdpMessage;

uint16_t lastPort;
ip4_addr_t lastIp;
///* Private function prototypes -----------------------------------------------*/
///* Private functions ---------------------------------------------------------*/
static void
udpecho_thread(void *arg)
{
  struct netconn *conn;
  struct netbuf *buf, *tx_buf;
  err_t err;
  LWIP_UNUSED_ARG(arg);

  conn = netconn_new(NETCONN_UDP);
  LWIP_ERROR("udpecho: invalid conn", (conn != NULL), return;);

  uint8_t ip[4];
  ip[0] = 224;
  ip[1] = ip[2] = 0;
  ip[3] = 3;
  ip4_addr_t ipAddr;
  IP4_ADDR(&ipAddr, ip[0], ip[1], ip[2], ip[3]);
  netconn_bind(conn, NULL, UDP_SERVER_PORT);

  // Wait until link is up
  while (!netif_is_link_up(&gnetif))
    osDelay(100);
  // !!! netconn_join_leave_group() MUST be called after link is up, or everything
  // silently fails (no error code but the unit cannot even be pinged) !!!
  netconn_join_leave_group(conn, &ipAddr, IP4_ADDR_ANY/*&localAddress*/, NETCONN_JOIN);

  while (1) {
    err = netconn_recv(conn, &buf);
    if (err == ERR_OK) {

      tx_buf = netbuf_new();
      // Prevent hard fault when dereferencing NULL pointer when allocation fails
      if (tx_buf != NULL)
      {
        uint8_t bufferTx[BUFFER_LENGTH], bufferRx[BUFFER_LENGTH], *bufferTmp = NULL;
        uint16_t lengthRx = 0, lengthTx = 0;

        if (netbuf_data(buf, (void**) &bufferTmp, &lengthRx) == ERR_OK)
        {
          memcpy(bufferRx, bufferTmp, lengthRx);

          if (CorrectMulticastReceived(bufferRx, &lengthRx, bufferTx, &lengthTx))
          {
            // Create new netbuf
            struct netbuf* sendingNetbuf = netbuf_new();
            // Allocate memory for packet buffer
            netbuf_alloc(sendingNetbuf, lengthTx);
            if (sendingNetbuf != NULL)
            {
              // Copy data to the netbuf
              pbuf_take(sendingNetbuf->p, (const void*) bufferTx, lengthTx);

              err = netconn_sendto(conn, sendingNetbuf, (const ip_addr_t *)&ipAddr/*(const ip_addr_t *)&(buf->addr)*/, buf->port);
              if(err != ERR_OK) {
                LWIP_DEBUGF(LWIP_DBG_ON, ("netconn_send failed: %d\n", (int)err));
              } else {
                LWIP_DEBUGF(LWIP_DBG_ON, ("got %s\n", bufferRx));
              }
              netbuf_free(sendingNetbuf);
              netbuf_delete(sendingNetbuf);
            }

            MulticastResponseSent(bufferRx, &lengthRx, bufferTx, &lengthTx);
          }
        }
        netbuf_delete(tx_buf);
      }
    }
    netbuf_delete(buf);
  }
}
/*----------------------------------------------------------------------------*/
static void udpCommunicationThread(void *arg)
{
  struct netconn *conn;
  struct netbuf *buf, *tx_buf;
  err_t err;
  LWIP_UNUSED_ARG(arg);

  conn = netconn_new(NETCONN_UDP);
  LWIP_ERROR("udpecho: invalid conn", (conn != NULL), return;);

  uint8_t* ip = GetConfiguration().IpAddress;
  ip4_addr_t ipAddr;
  IP4_ADDR(&ipAddr, ip[0], ip[1], ip[2], ip[3]);
  netconn_bind(conn, &ipAddr, GetConfiguration().Port);
  // Wait until link is up
  while (!netif_is_link_up(&gnetif))
    osDelay(100);

  while (1) {
    err = netconn_recv(conn, &buf);
    if (err == ERR_OK) {

      ip4_addr_copy(lastIp, buf->addr);
      lastPort = buf->port;
      tx_buf = netbuf_new();
      // Prevent hard fault when dereferencing NULL pointer when allocation fails
      if (tx_buf != NULL)
      {
        uint8_t bufferRx[BUFFER_LENGTH], *bufferTmp = NULL;
        uint16_t lengthRx = 0;

        if (netbuf_data(buf, (void**) &bufferTmp, &lengthRx) == ERR_OK)
        {
          memcpy(bufferRx, bufferTmp, lengthRx);
          UdpDataReceived(bufferRx, lengthRx);
        }
        netbuf_delete(tx_buf);
      }
    }
    netbuf_delete(buf);
  }
}
/*----------------------------------------------------------------------------*/
osStatus_t UdpEnqueueResponse(uint8_t* pData, uint16_t length)
{
    GenericMessageType msg;
    msg.Datalen = length > MAX_CMD_LEN ? MAX_CMD_LEN : length;
    length = msg.Datalen;
    for (uint16_t i = 0; i < length; i++)
        msg.Data[i] = pData[i];
    /* Determine if called from interrupt */
    uint32_t timeout = (__get_IPSR() != 0U) ? 0 : QUEUE_PUT_TIMEOUT;
    return osMessageQueuePut(UdpTxQueueHandle, &msg, 0, timeout);
}
/*----------------------------------------------------------------------------*/
static void UdpSendThread(void* arg)
{
    (void) arg;

    struct netconn *conn = netconn_new(NETCONN_UDP);

    osStatus_t queueState;
    GenericMessageType udpTransmitBuffer;
    while (1)
    {
      while ((queueState = osMessageQueueGet(UdpTxQueueHandle, &udpTransmitBuffer, NULL, 0xffff)) != osOK);

      // Create new netbuf
      struct netbuf* sendingNetbuf = netbuf_new();
      // Allocate memory for packet buffer
      netbuf_alloc(sendingNetbuf, udpTransmitBuffer.Datalen);

      if (sendingNetbuf != NULL)
      {
          // Copy data to the netbuf
          pbuf_take(sendingNetbuf->p, (const void*) udpTransmitBuffer.Data, udpTransmitBuffer.Datalen);
          err_t err = netconn_sendto(conn, sendingNetbuf, (const ip_addr_t *)&(lastIp), lastPort);

          if (err != ERR_OK)
              LWIP_DEBUGF(LWIP_DBG_ON, ("netconn_send failed: %d\n", (int)err));
          netbuf_free(sendingNetbuf);
          netbuf_delete(sendingNetbuf);
      }
    }
}
/*----------------------------------------------------------------------------*/
void
UdpServerInit(void)
{
  sys_thread_new("udpecho_thread", udpecho_thread, NULL, (configMINIMAL_STACK_SIZE * 12), UDPECHO_THREAD_PRIO);
  sys_thread_new("UDP prot. thread", udpCommunicationThread, NULL, (configMINIMAL_STACK_SIZE * 42), UDP_PROTOCOL_THREAD_PRIO);
  sys_thread_new("UDP send thread", UdpSendThread, NULL, (configMINIMAL_STACK_SIZE * 12), UDPSEND_THREAD_PRIO);
}

#endif /* LWIP_NETCONN */
