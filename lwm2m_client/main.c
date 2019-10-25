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

#define ENDPOINT_NAME "nRF52_LwM2M_client"

enum lwm2m_int_type {
  LWM2M_INT_TYPE_BOOLEAN,
  LWM2M_INT_TYPE_UINT8,
  LWM2M_INT_TYPE_UINT16,
  LWM2M_INT_TYPE_UINT32,
  LWM2M_INT_TYPE_UINT64,
  LWM2M_INT_TYPE_INT8,
  LWM2M_INT_TYPE_INT16,
  LWM2M_INT_TYPE_INT32,
  LWM2M_INT_TYPE_INT64,
};

/**
 * @brief LwM2M Objects managed by OMA for LwM2M tech specification.  Objects
 * in this range have IDs from 0 to 1023.
 * For more information refer to Technical Specification
 * OMA-TS-LightweightM2M-V1_0_2-20180209-A
 */

#define LWM2M_OBJECT_SECURITY_ID                  0
#define LWM2M_OBJECT_SERVER_ID                    1
#define LWM2M_OBJECT_ACCESS_CONTROL_ID            2
#define LWM2M_OBJECT_DEVICE_ID                    3
#define LWM2M_OBJECT_CONNECTIVITY_MONITORING_ID   4
#define LWM2M_OBJECT_FIRMWARE_ID                  5
#define LWM2M_OBJECT_LOCATION_ID                  6
#define LWM2M_OBJECT_CONNECTIVITY_STATISTICS_ID   7

/**
 * @brief LwM2M Objects produced by 3rd party Standards Development
 * Organizations.  Objects in this range have IDs from 2048 to 10240
 * Refer to the OMA LightweightM2M (LwM2M) Object and Resource Registry:
 * http://www.openmobilealliance.org/wp/OMNA/LwM2M/LwM2MRegistry.html
 */

#define IPSO_OBJECT_TEMP_SENSOR_ID                3303
#define IPSO_OBJECT_LIGHT_CONTROL_ID              3311
#define IPSO_OBJECT_TIMER_ID                      3340

/**
 * @brief LwM2M RD client events
 *
 * LwM2M client events are passed back to the event_cb function in
 * lwm2m_rd_client_start()
 */
enum lwm2m_rd_client_event {
	LWM2M_RD_CLIENT_EVENT_NONE,
	LWM2M_RD_CLIENT_EVENT_BOOTSTRAP_REG_FAILURE,
	LWM2M_RD_CLIENT_EVENT_BOOTSTRAP_REG_COMPLETE,
	LWM2M_RD_CLIENT_EVENT_BOOTSTRAP_TRANSFER_COMPLETE,
	LWM2M_RD_CLIENT_EVENT_REGISTRATION_FAILURE,
	LWM2M_RD_CLIENT_EVENT_REGISTRATION_COMPLETE,
	LWM2M_RD_CLIENT_EVENT_REG_UPDATE_FAILURE,
	LWM2M_RD_CLIENT_EVENT_REG_UPDATE_COMPLETE,
	LWM2M_RD_CLIENT_EVENT_DEREGISTER_FAILURE,
	LWM2M_RD_CLIENT_EVENT_DISCONNECT
};

static bool lwm2m_connected = false;
APP_TIMER_DEF(m_timer);

/**@brief Function for handling bsp events.
 */
void bsp_evt_handler(bsp_event_t evt)
{
    switch (evt)
    {
        case BSP_EVENT_KEY_0:
            if (!lwm2m_connected) {
              /*param format [endpoint_name(var)]*/
              inter_connect_send(CMD_TYPE_LWM2M_CONNECT, ENDPOINT_NAME, sizeof(ENDPOINT_NAME)-1);
            }
            else
            {
              inter_connect_send(CMD_TYPE_LWM2M_DISCONNECT, NULL, 0);
            }
            break;

        case BSP_EVENT_KEY_1:
            if (lwm2m_connected)
            {
              /*param format [integer type(1)][value(0~4)][path(var)]*/
              char data[] = "03311/0/5850";
              data[0] = LWM2M_INT_TYPE_BOOLEAN;
              inter_connect_send(CMD_TYPE_LWM2M_READ_INT, (uint8_t *)data, sizeof(data));
            }
            break;

        case BSP_EVENT_KEY_2:
            if (lwm2m_connected)
            {
              bsp_board_led_invert(0);
              bool state = bsp_board_led_state_get(0);
              /*param format [integer type(1)][value(0~4)][path(var)]*/
              char data[] = "003311/0/5850";
              data[0] = LWM2M_INT_TYPE_BOOLEAN;
              data[1] = state;
              inter_connect_send(CMD_TYPE_LWM2M_WRITE_INT, (uint8_t *)data, sizeof(data));
            }
            break;

        case BSP_EVENT_KEY_3:
            if (lwm2m_connected)
            {
              bsp_board_led_invert(1);
              bool state = bsp_board_led_state_get(1);
              /*param format [integer type(1)][value(0~4)][path(var)]*/
              char data[] = "003311/1/5850";
              data[0] = LWM2M_INT_TYPE_BOOLEAN;
              data[1] = state;
              inter_connect_send(CMD_TYPE_LWM2M_WRITE_INT, (uint8_t *)data, sizeof(data));
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

static void handle_object_event(uint16_t obj_id, uint16_t ins_id, const uint8_t *data)
{
  NRF_LOG_INFO("Handle event for Object %d, Instance %d", obj_id, ins_id);
  if (obj_id == IPSO_OBJECT_LIGHT_CONTROL_ID)
  {
    if (*data) 
      bsp_board_led_on(ins_id);
    else 
      bsp_board_led_off(ins_id);
  }
}  

/**@snippet [Handling the data received over UART] */
static void serial_data_handler(uint8_t data_type, const uint8_t *data_buf, uint8_t data_len)
{
  NRF_LOG_DEBUG("Processing 0x%02x, len:%d", data_type, data_len);
  switch (data_type)
  {
    case CMD_TYPE_DATA:
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

    // LwM2M
    case RSP_TYPE_BASE|CMD_TYPE_LWM2M_CONNECT:
    case RSP_TYPE_BASE|CMD_TYPE_LWM2M_DISCONNECT:
      break;
    case RSP_TYPE_BASE|CMD_TYPE_LWM2M_WRITE_INT:
      NRF_LOG_INFO("LwM2M write return: %d", *data_buf);
      break;
    case RSP_TYPE_BASE|CMD_TYPE_LWM2M_READ_INT:
      NRF_LOG_INFO("LwM2M read value: %d", *data_buf);
      break;
    
    case RSP_TYPE_NOTIFICATION:
    {
      uint8_t type = *data_buf;
      if (type == NOT_TYPE_LWM2M_RD)
      {
        NRF_LOG_INFO("LwM2M RD event received");
        uint8_t evt = *(data_buf + 1);
        if (evt == LWM2M_RD_CLIENT_EVENT_REGISTRATION_COMPLETE)
        {
          NRF_LOG_INFO("Connected");
          lwm2m_connected = true;
        }
        if (evt == LWM2M_RD_CLIENT_EVENT_DISCONNECT)
        {
          NRF_LOG_INFO("Disconnected");
          lwm2m_connected = false;
        }
      }
      if (type == NOT_TYPE_LWM2M_OBJECT)
      {
        uint16_t obj_id = (*(data_buf + 1)) << 8;
        uint16_t ins_id = (*(data_buf + 3)) << 8;

        NRF_LOG_INFO("LwM2M object event received");
        obj_id += *(data_buf + 2);
        ins_id += *(data_buf + 4);
        handle_object_event(obj_id, ins_id, (uint8_t *)(data_buf+5));
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

    NRF_LOG_INFO("LwM2M client started.");
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
