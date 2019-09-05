/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
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
/** @file
 *
 * @defgroup bsp_example_main main.c
 * @{
 * @ingroup bsp_example
 * @brief BSP Example Application main file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "boards.h"
#include "bsp.h"
#include "app_timer.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include "nrf_gpio.h"

#include "rtt_input.h"
#include "inter_connect.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define WAKEUP_GPIO_PIN   27

/**
 * @brief MQTT Asynchronous Events notified to the application from the module
 *        through the callback registered by the application.
 */
enum mqtt_evt_type {
	/** Acknowledgment of connection request. Event result accompanying
	 *  the event indicates whether the connection failed or succeeded.
	 */
	MQTT_EVT_CONNACK,

	/** Disconnection Event. MQTT Client Reference is no longer valid once
	 *  this event is received for the client.
	 */
	MQTT_EVT_DISCONNECT,

	/** Publish event received when message is published on a topic client
	 *  is subscribed to.
	 */
	MQTT_EVT_PUBLISH,

	/** Acknowledgment for published message with QoS 1. */
	MQTT_EVT_PUBACK,

	/** Reception confirmation for published message with QoS 2. */
	MQTT_EVT_PUBREC,

	/** Release of published message with QoS 2. */
	MQTT_EVT_PUBREL,

	/** Confirmation to a publish release message with QoS 2. */
	MQTT_EVT_PUBCOMP,

	/** Acknowledgment to a subscribe request. */
	MQTT_EVT_SUBACK,

	/** Acknowledgment to a unsubscribe request. */
	MQTT_EVT_UNSUBACK
};

/** @brief MQTT version protocol level. */
enum mqtt_version {
	MQTT_VERSION_3_1_0 = 3, /**< Protocol level for 3.1.0. */
	MQTT_VERSION_3_1_1 = 4  /**< Protocol level for 3.1.1. */
};

/** @brief MQTT Quality of Service types. */
enum mqtt_qos {
	/** Lowest Quality of Service, no acknowledgment needed for published
	 *  message.
	 */
	MQTT_QOS_0_AT_MOST_ONCE = 0x00,

	/** Medium Quality of Service, if acknowledgment expected for published
	 *  message, duplicate messages permitted.
	 */
	MQTT_QOS_1_AT_LEAST_ONCE = 0x01,

	/** Highest Quality of Service, acknowledgment expected and message
	 *  shall be published only once. Message not published to interested
	 *  parties unless client issues a PUBREL.
	 */
	MQTT_QOS_2_EXACTLY_ONCE  = 0x02
};

/** @brief MQTT CONNACK return codes. */
enum mqtt_conn_return_code {
	/** Connection accepted. */
	MQTT_CONNECTION_ACCEPTED                = 0x00,

	/** The Server does not support the level of the MQTT protocol
	 * requested by the Client.
	 */
	MQTT_UNACCEPTABLE_PROTOCOL_VERSION      = 0x01,

	/** The Client identifier is correct UTF-8 but not allowed by the
	 *  Server.
	 */
	MQTT_IDENTIFIER_REJECTED                = 0x02,

	/** The Network Connection has been made but the MQTT service is
	 *  unavailable.
	 */
	MQTT_SERVER_UNAVAILABLE                 = 0x03,

	/** The data in the user name or password is malformed. */
	MQTT_BAD_USER_NAME_OR_PASSWORD          = 0x04,

	/** The Client is not authorized to connect. */
	MQTT_NOT_AUTHORIZED                     = 0x05
};

/** @brief MQTT SUBACK return codes. */
enum mqtt_suback_return_code {
	/** Subscription with QoS 0 succeeded. */
	MQTT_SUBACK_SUCCESS_QoS_0 = 0x00,

	/** Subscription with QoS 1 succeeded. */
	MQTT_SUBACK_SUCCESS_QoS_1 = 0x01,

	/** Subscription with QoS 2 succeeded. */
	MQTT_SUBACK_SUCCESS_QoS_2 = 0x02,

	/** Subscription for a topic failed. */
	MQTT_SUBACK_FAILURE = 0x80
};

static bool mqtt_connected = false;
APP_TIMER_DEF(m_timer);

#define CLIENT_ID   "nRF52_MQTT_client"
#define SUB_TOPIC   "/my/test/topic"
#define PUB_TOPIC   "/my/test/topic"

static uint8_t mqtt_buf[128];

/**@brief Function for handling bsp events.
 */
void bsp_evt_handler(bsp_event_t evt)
{
    static bool sub_unsub = true;
    static uint8_t pub_data = 0; 

    switch (evt)
    {
        case BSP_EVENT_KEY_0:
            if (!mqtt_connected) {
              /*param format [client_id(var)]*/
              inter_connect_send(CMD_TYPE_MQTT_CONNECT, CLIENT_ID, sizeof(CLIENT_ID)-1);
            }
            break;

        case BSP_EVENT_KEY_1:
            if (mqtt_connected) {
              inter_connect_send(CMD_TYPE_MQTT_DISCONNECT, NULL, 0);
            }
            break;

        case BSP_EVENT_KEY_2:
            if (mqtt_connected) 
            {
              if (sub_unsub) 
              {
                /*param format [QoS(1)][length(1)][topic(var)]*/
                memset(mqtt_buf, 0x00, sizeof(mqtt_buf));
//                mqtt_buf[0] = MQTT_QOS_0_AT_MOST_ONCE;
                mqtt_buf[0] = MQTT_QOS_1_AT_LEAST_ONCE;
//                mqtt_buf[0] = MQTT_QOS_2_EXACTLY_ONCE;
                mqtt_buf[1] = sizeof(SUB_TOPIC)-1;
                memcpy(&mqtt_buf[2], SUB_TOPIC, sizeof(SUB_TOPIC)-1);
                inter_connect_send(CMD_TYPE_MQTT_SUBSCRIBE, mqtt_buf, sizeof(SUB_TOPIC)+1);
              }
              else
              {
                /*param format [topic(var)]*/ 
                inter_connect_send(CMD_TYPE_MQTT_UNSUBSCRIBE, SUB_TOPIC, sizeof(SUB_TOPIC)-1);
              }
              sub_unsub = !sub_unsub;
            }
            break;

        case BSP_EVENT_KEY_3:
            if (mqtt_connected)
            {
                /*param format [QoS(1)][length(1)][topic(var)][length(1)][data(var)]*/
                uint8_t size_topic = sizeof(PUB_TOPIC)-1;

                memset(mqtt_buf, 0x00, sizeof(mqtt_buf));
//                mqtt_buf[0] = MQTT_QOS_0_AT_MOST_ONCE;
                mqtt_buf[0] = MQTT_QOS_1_AT_LEAST_ONCE;
//                mqtt_buf[0] = MQTT_QOS_2_EXACTLY_ONCE;
                mqtt_buf[1] = size_topic;
                memcpy(&mqtt_buf[2], PUB_TOPIC, size_topic);
                mqtt_buf[2+size_topic] = 0x01;
                mqtt_buf[3+size_topic] = pub_data;
                inter_connect_send(CMD_TYPE_MQTT_PUBLISH, mqtt_buf, size_topic+4);
              
                pub_data++;
            }
            break;
            
        default:
            return; // no implementation needed
    }
}


/**@brief Function for initializing low frequency clock.
 */
void clock_initialization()
{
    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }
}


/**@brief Function for initializing bsp module.
 */
void bsp_configuration()
{
    uint32_t err_code;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@snippet [Handling the data received over UART] */
static void serial_data_handler(uint8_t data_type, const uint8_t *data_buf, uint8_t data_len)
{
  NRF_LOG_DEBUG("Processing 0x%02x, len:%d", data_type, data_len);
  switch (data_type)
  {
    case CMD_TYPE_DATA:
    case RSP_TYPE_BASE|CMD_TYPE_AT_CMD:
      NRF_LOG_INFO("%s", data_buf);
      break;

    case CMD_TYPE_SYNC_CMD:
      app_timer_stop(m_timer);
      NRF_LOG_INFO("Sync up");
      inter_connect_send(RSP_TYPE_BASE|CMD_TYPE_SYNC_CMD, NULL, 0);
      nrf_gpio_pin_set(WAKEUP_GPIO_PIN);
      NRF_LOG_INFO("Connect LTE");
      inter_connect_send(CMD_TYPE_MDM_INT_CONNECT, NULL, 0);
      break;

    // Modem control
    case RSP_TYPE_BASE|CMD_TYPE_MDM_INT_CONNECT:
      if (*data_buf == 0)
      {
        NRF_LOG_INFO("LTE connected");
        inter_connect_send(CMD_TYPE_TIME_CMD, NULL, 0);
      }
      else
      {
        NRF_LOG_INFO("LTE connection failed (%d)", *data_buf);
      }      
      break;
      
    // Current GMT:
    case RSP_TYPE_BASE|CMD_TYPE_TIME_CMD:
    {
        int8_t result = *data_buf;
        if (result == 0)
        {
          int year, month, day;
          int hour, minute, second;
          //uint8_t enable_gps = 1;

          memcpy(&year, (data_buf + 1), 4);
          memcpy(&month, (data_buf + 5), 4);
          memcpy(&day, (data_buf + 9), 4);
          memcpy(&hour, (data_buf + 13), 4);
          memcpy(&minute, (data_buf + 17), 4);
          memcpy(&second, (data_buf + 21), 4);

          NRF_LOG_INFO("GMT time: %04d/%02d/%02d %02d:%02d:%02d", 
            year, month, day, hour, minute, second);

          // Enabled GPS
          //inter_connect_send(CMD_TYPE_GPS_CMD, &enable_gps, 1);
        }
    }
    break;

    // GPS
    case RSP_TYPE_BASE|CMD_TYPE_GPS_CMD:
    {
      NRF_LOG_INFO("GPS fix");
    }
    break;
    
    // MQTT
    case RSP_TYPE_BASE|CMD_TYPE_MQTT_CONNECT:
    case RSP_TYPE_BASE|CMD_TYPE_MQTT_DISCONNECT:
    case RSP_TYPE_BASE|CMD_TYPE_MQTT_SUBSCRIBE:
    case RSP_TYPE_BASE|CMD_TYPE_MQTT_UNSUBSCRIBE:
    case RSP_TYPE_BASE|CMD_TYPE_MQTT_PUBLISH:
      break;
    
    case RSP_TYPE_NOTIFICATION:
    {
      uint8_t type = *data_buf;
      if (type == NOT_TYPE_AT)
      {
        NRF_LOG_INFO("AT unsolicited data received");
      }
      else if (type == NOT_TYPE_MQTT)
      {
        NRF_LOG_INFO("MQTT unsolicited data received");
        uint8_t evt = *(data_buf + 1);
        uint8_t res = *(data_buf + 2);
        if (evt == MQTT_EVT_CONNACK)
        {
          NRF_LOG_INFO("Connect result: %d", res);
          mqtt_connected = true;
        }
        if (evt == MQTT_EVT_DISCONNECT)
        {
          NRF_LOG_INFO("Disconnect result: %d", res);
          mqtt_connected = false;
        }
        if (evt == MQTT_EVT_SUBACK)
        {
          NRF_LOG_INFO("Subscribe result: %d", res);
        }
        if (evt == MQTT_EVT_UNSUBACK)
        {
          NRF_LOG_INFO("Unsubscribe result: %d", res);
        }
        if (evt == MQTT_EVT_PUBACK)
        {
          NRF_LOG_INFO("Publish result: %d", res);
        }
        if (evt == MQTT_EVT_PUBLISH)
        {
          NRF_LOG_INFO("PUB received: %d", res);
        }
      }
    } break;

    default:
      NRF_LOG_INFO("cmd: %d rsp: %d", data_type & (~RSP_TYPE_BASE), (int8_t)(*data_buf));
      break;
  }
}


static void timer_handle(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    nrf_gpio_pin_toggle(WAKEUP_GPIO_PIN);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    clock_initialization();

    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    err_code = inter_connect_init(serial_data_handler);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("MQTT client started.");
    bsp_configuration();

    err_code = app_timer_create(&m_timer, APP_TIMER_MODE_REPEATED, timer_handle);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_timer, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);
    // wake up nRF91 now
    nrf_gpio_cfg_output(WAKEUP_GPIO_PIN);
    nrf_gpio_pin_clear(WAKEUP_GPIO_PIN);

    while (true)
    {
        NRF_LOG_FLUSH();
        __SEV();
        __WFE();
        __WFE();
        // no implementation needed
    }
}


/** @} */
