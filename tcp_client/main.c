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
#include "inter_connect.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define WAKEUP_GPIO_PIN   27

static bool tcp_connected = false;
APP_TIMER_DEF(m_timer);

#define SERVER_NAME   "google.com"
#define SERVER_IP     "104.28.28.162"
#define SERVER_PORT   80
#define TEST_DATA     "GET / HTTP/1.1\r\nHost: google.com\r\n\r\n"

static uint8_t tcp_buf[128];

/**@brief Function for handling bsp events.
 */
void bsp_evt_handler(bsp_event_t evt)
{
    uint16_t port = SERVER_PORT;

    switch (evt)
    {
        case BSP_EVENT_KEY_0:
            if (!tcp_connected) {
              /* param format [port(2)][hostname or ip_address]*/
              memcpy(tcp_buf, &port, 2);
              memcpy(tcp_buf+2, SERVER_NAME, sizeof(SERVER_NAME)-1);
              inter_connect_send(CMD_TYPE_TCP_CONNECT, tcp_buf, sizeof(SERVER_NAME)+1);
            }
            break;

        case BSP_EVENT_KEY_1:
            if (!tcp_connected) {
              /* param format [port(2)][by_ip(1)][hostname or ip_address]*/
              memcpy(tcp_buf, &port, 2);
              memcpy(tcp_buf+2, SERVER_IP, sizeof(SERVER_IP)-1);
              inter_connect_send(CMD_TYPE_TCP_CONNECT, tcp_buf, sizeof(SERVER_IP)+1);
            }
            break;

        case BSP_EVENT_KEY_2:
            if (tcp_connected) 
            {
                inter_connect_send(CMD_TYPE_TCP_DISCONNECT, NULL, 0);
            }
            break;

        case BSP_EVENT_KEY_3:
            if (tcp_connected)
            {
                inter_connect_send(CMD_TYPE_TCP_SEND, TEST_DATA, sizeof(TEST_DATA)-1);
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

          memcpy(&year, (data_buf + 1), 4);
          memcpy(&month, (data_buf + 5), 4);
          memcpy(&day, (data_buf + 9), 4);
          memcpy(&hour, (data_buf + 13), 4);
          memcpy(&minute, (data_buf + 17), 4);
          memcpy(&second, (data_buf + 21), 4);

          NRF_LOG_INFO("GMT time: %04d/%02d/%02d %02d:%02d:%02d", 
            year, month, day, hour, minute, second);
        }
    }
    break;

    // TCP
    case RSP_TYPE_BASE|CMD_TYPE_TCP_CONNECT:
      if (*data_buf == 0)
      {
        NRF_LOG_INFO("TCP connected");
        tcp_connected = true;
      }
      else
      {
        NRF_LOG_INFO("TCP connection failed (%d)", *data_buf);
      }      
      break;
    case RSP_TYPE_BASE|CMD_TYPE_TCP_DISCONNECT:
      if (*data_buf == 0)
      {
        NRF_LOG_INFO("TCP disconnected");
        tcp_connected = false;
      }
      else
      {
        NRF_LOG_INFO("TCP disconnection failed (%d)", *data_buf);
      }
      break;
    case RSP_TYPE_BASE|CMD_TYPE_TCP_SEND:
      if (*data_buf == 0)
      {
        NRF_LOG_INFO("TCP Send OK");
        inter_connect_send(CMD_TYPE_TCP_RECEIVE, NULL, 0);
      }
      break;
    
    case RSP_TYPE_NOTIFICATION:
    {
      uint8_t type = *data_buf;
      if (type == NOT_TYPE_AT)
      {
        NRF_LOG_INFO("AT unsolicited data received");
      }
      else if (type == NOT_TYPE_TCP)
      {
        NRF_LOG_INFO("TCP unsolicited data received");
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

    NRF_LOG_INFO("TCP client started.");
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
