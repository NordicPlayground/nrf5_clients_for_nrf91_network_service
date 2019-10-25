/**
 * Copyright (c) 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/**@file
 *
 * @note implementation of simple control protol over UART
 */

#ifndef INTER_CONNECT_H__
#define INTER_CONNECT_H__

#include "nrf.h"
#include "sdk_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Command type. */
enum ic_cmd_type {
  /** Arbitary data. */
  CMD_TYPE_DATA = 0x00,
  /** AT command service. */
	CMD_TYPE_GENERIC_BASE = 0x01,
  CMD_TYPE_SYNC_CMD = CMD_TYPE_GENERIC_BASE,
  CMD_TYPE_TIME_CMD,
  /** Modem control service */
  CMD_TYPE_MDM_BASE = 0x10,
  CMD_TYPE_MDM_INT_CONNECT = CMD_TYPE_MDM_BASE,
  CMD_TYPE_MDM_GO_OFFLINE,
  CMD_TYPE_MDM_POWER_OFF,
  CMD_TYPE_MDM_GO_ONLINE,
  CMD_TYPE_MDM_PSM_REQ,
  CMD_TYPE_MDM_EDRX_REQ,
  /** MQTT service **/
  CMD_TYPE_MQTT_BASE = 0x20,
  CMD_TYPE_MQTT_CONNECT = CMD_TYPE_MQTT_BASE,
  CMD_TYPE_MQTT_DISCONNECT,
  CMD_TYPE_MQTT_SUBSCRIBE,
  CMD_TYPE_MQTT_UNSUBSCRIBE,
  CMD_TYPE_MQTT_PUBLISH, 
  /** CoAP service **/
  CMD_TYPE_COAP_BASE = 0x30,
  /** LWM2M service **/
  CMD_TYPE_LWM2M_BASE = 0x40,
  CMD_TYPE_LWM2M_CONNECT = CMD_TYPE_LWM2M_BASE,
  CMD_TYPE_LWM2M_DISCONNECT,
  CMD_TYPE_LWM2M_READ_INT,
  CMD_TYPE_LWM2M_WRITE_INT,
  CMD_TYPE_LWM2M_READ_FLOAT,
  CMD_TYPE_LWM2M_WRITE_FLOAT,
  CMD_TYPE_LWM2M_READ_STRING,
  CMD_TYPE_LWM2M_WRITE_STRING,
  CMD_TYPE_LWM2M_READ_OPAQUE,
  CMD_TYPE_LWM2M_WRITE_OPAQUE,
  /** GPS service **/
  CMD_TYPE_GPS_BASE = 0x50,
  CMD_TYPE_GPS_CONTROL = CMD_TYPE_GPS_BASE,
  CMD_TYPE_GPS_STATE,
	/** TCP/IP service **/
  CMD_TYPE_TCPIP_BASE = 0x60,
  CMD_TYPE_TCP_CONNECT = CMD_TYPE_TCPIP_BASE,
  CMD_TYPE_TCP_DISCONNECT,
  CMD_TYPE_TCP_SEND,
  CMD_TYPE_TCP_RECEIVE,
  CMD_TYPE_UDP_SENDTO,
  CMD_TYPE_UDP_RECEIVEFROM,
  /** Type reserved. */
  CMD_TYPE_RESERVED  = 0x7F,
  /** Type response base. */
  RSP_TYPE_BASE  = 0x80,
  /** Type unsocilicted notification */
  RSP_TYPE_NOTIFICATION = 0xFF
};

/** Unsolicited notification type. */
enum ic_notify_type {
  NOT_TYPE_BASE   = 0x00,

  /** Notification type AT */
  NOT_TYPE_AT     = 0x01,
  /** Notification type Modem Control */
  NOT_TYPE_MDM,
  /** Notification type MQTT */
  NOT_TYPE_MQTT,
  /** Notification type CoAP */
  NOT_TYPE_COAP,
  /** Notification type LwM2M */
  NOT_TYPE_LWM2M_RD,          // Registration and discovery events
  NOT_TYPE_LWM2M_OBJECT,      // LwM2M Object events
  /** Notification type GPS */
  NOT_TYPE_GPS,
	/** Notification type TCP */
	NOT_TYPE_TCP,
	/** Notification type UDP */
	NOT_TYPE_UDP,

  NOT_TYPE_INVALID
};

/**
 * @typedef data_handler_t
 * @brief Callback when data is received from inter_connect interface.
 *
 * @param data_type Type of data, see ic_cmd_type.
 * @param data_buf  Data buffer.
 * @param data_len  Length of data buffer.
 */
typedef void (*data_handler_t)(uint8_t data_type, const uint8_t *data_buf, uint8_t data_len);

/** @brief Initialize the library.
 *
 *  @param  data_handler Callback handler for received data.
 *
 *  @retval 0           If the operation was successful.
 *                      Otherwise, a (negative) error code is returned.
 */
uint32_t inter_connect_init(data_handler_t data_handler);

/** @brief Uninitialize the library.
 *
 *
 *  @retval 0           If the operation was successful.
 *                      Otherwise, a (negative) error code is returned.
 */
uint32_t inter_connect_uninit(void);

/** @brief Send data to the peer.
 *
 *  @param data_type Type of data, see ic_cmd_type.
 *  @param data_buf  Data buffer.
 *  @param data_len  Length of data buffer.
 *
 *  @retval 0           If the operation was successful.
 *                      Otherwise, a (negative) error code is returned.
 */
uint32_t inter_connect_send(uint8_t data_type, const uint8_t *data_buf, uint8_t data_len);

#ifdef __cplusplus
}
#endif

#endif // INTER_CONNECT_H__
