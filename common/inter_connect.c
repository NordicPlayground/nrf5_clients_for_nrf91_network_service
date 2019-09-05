/**
 * Copyright (c) 2019, Nordic Semiconductor ASA
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
 * This file contains the source code for a control protocol over UART.
 */

#include <ctype.h>
#include "nrf.h"
#include "bsp.h"
#include "nrf_serial.h"

#define NRF_LOG_MODULE_NAME prot
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();

#include "inter_connect.h"

/**
 * Data packet format
 * [STX(1)][TYPE(1)][LENTH(1)][VALUE(vary)][BCC(1)]
 */
#define PROT_HEADER_STX  0x02  /** STX char */
#define PROT_HEADER_LEN  3     /** [STX][TYPE][LENGTH]*/
#define PROT_OVEREAD     PROT_HEADER_LEN + 1     /** [STX][TYPE][LENGTH]...[BCC]*/
#define PROT_STX_POS     0     /** Position of STX byte */
#define PROT_TYP_POS     1     /** Position of Type byte */
#define PROT_LEN_POS     2     /** Position of Length byte */

#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE)
/**< Max payload in ATT_MTU plus prorocol overheade */
#define DEV_CTRL_MAX_DATA_LEN   (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 3 + PROT_OVEREAD) 
#else
#define DEV_CTRL_MAX_DATA_LEN   (128 - 3 + PROT_OVEREAD) 
#endif

/**< Serial port definitions */
#define SERIAL_FIFO_TX_SIZE     DEV_CTRL_MAX_DATA_LEN * 2
#define SERIAL_FIFO_RX_SIZE     DEV_CTRL_MAX_DATA_LEN * 2
#define SERIAL_BUFF_TX_SIZE     DEV_CTRL_MAX_DATA_LEN    /**< bulk sending */
#define SERIAL_BUFF_RX_SIZE     1                        /**< byte-by-byte receiving */

NRF_SERIAL_DRV_UART_CONFIG_DEF(uart0_drv_config,
                      RX_PIN_NUMBER, TX_PIN_NUMBER,
                      RTS_PIN_NUMBER, CTS_PIN_NUMBER,
                      NRF_UART_HWFC_ENABLED, NRF_UART_PARITY_EXCLUDED,
                      NRF_UART_BAUDRATE_115200,
                      UART_DEFAULT_CONFIG_IRQ_PRIORITY);
NRF_SERIAL_QUEUES_DEF(serial_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);
NRF_SERIAL_BUFFERS_DEF(serial_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);

void on_serial_evt(struct nrf_serial_s const * p_serial, nrf_serial_event_t event);

NRF_SERIAL_CONFIG_DEF(serial_config, NRF_SERIAL_MODE_IRQ,
                      &serial_queues, &serial_buffs, on_serial_evt, NULL);
NRF_SERIAL_UART_DEF(m_serial_uart, 0);

typedef struct
{
    uint8_t           length;                            /* actuall data length of this buffer block */
    uint8_t           data[DEV_CTRL_MAX_DATA_LEN];       /* max length of one data buffer */
} uart_buf_block_t;

static uart_buf_block_t         m_tx_buf;
static uart_buf_block_t         m_rx_buf;
static uint8_t                  m_rx_size     = 0;

static data_handler_t data_handler_cb = NULL;

void uart_rx_init(void)
{
  memset(m_rx_buf.data, 0x00, DEV_CTRL_MAX_DATA_LEN);
  m_rx_buf.length = 0x00;
  m_rx_size = 0;
}

//
//  Device control protocol
//
#if APP_CFG_ENABLE_BCC
bool bcc_verify(uint8_t * p_data, uint32_t length)
{
  uint8_t *ptr;     
  uint8_t bcc;
  uint32_t i;

  ptr = p_data;                           // first byte STX
  for (i=OFFSET_ID; i<(length-1); i++)    // exclude STX and BCC
  {
    bcc ^= *(ptr+i);
  }
  
  return (bcc == *(ptr+i));               // last byte BCC
}
#endif

void uart_data_receive(uart_buf_block_t * p_rx_buf)
{
  bool packet_error = false;

  NRF_LOG_INFO("UART RECEIVE, length:%d.", p_rx_buf->length);
  NRF_LOG_HEXDUMP_INFO(p_rx_buf->data, p_rx_buf->length);

  // check length
  if (p_rx_buf->length < PROT_OVEREAD ||
      p_rx_buf->length > DEV_CTRL_MAX_DATA_LEN)
  {
    NRF_LOG_ERROR("Protocol length error");
    packet_error = true;
  }
#if APP_CFG_ENABLE_BCC
  // BCC check
  else if (!bcc_verify(data_array, p_rx_buf->length)) 
  {
    NRF_LOG_ERROR("BCC error");
    packet_error = true;
  }
#endif
  if (!packet_error)
  {
    data_handler_cb(m_rx_buf.data[PROT_TYP_POS],
                    &m_rx_buf.data[PROT_LEN_POS+1],
                    m_rx_buf.data[PROT_LEN_POS]);
  }
  // prepare for next receive
  uart_rx_init();
}

static void serial_receive_data(nrf_serial_t const * p_serial)
{
  // Invalid protocol packet size
  static uint8_t size_to_receive = DEV_CTRL_MAX_DATA_LEN+1;

  nrf_queue_t const * p_rxq = p_serial->p_ctx->p_config->p_queues->p_rxq;
  m_rx_size += nrf_queue_out(p_rxq, &m_rx_buf.data[m_rx_size], SERIAL_BUFF_RX_SIZE);
  if (m_rx_size == (PROT_LEN_POS+1))
  {
    size_to_receive = m_rx_buf.data[PROT_LEN_POS] + PROT_OVEREAD; 
    NRF_LOG_DEBUG("Size: %d", size_to_receive);
    if (size_to_receive > DEV_CTRL_MAX_DATA_LEN)
    {
      NRF_LOG_ERROR("Wrong length received: %d", m_rx_buf.data[PROT_LEN_POS]);
      size_to_receive = PROT_OVEREAD;
      // TO-DO reset UART RX by GPIO
    }
  }
  if (m_rx_size >= size_to_receive)
  {
    m_rx_buf.length = size_to_receive;
    uart_data_receive(&m_rx_buf);    
  }
}

static void on_serial_evt(nrf_serial_t const * p_serial, nrf_serial_event_t event)
{
  switch (event)
  {
    case NRF_SERIAL_EVENT_TX_DONE:
      NRF_LOG_DEBUG("NRF_SERIAL_EVENT_TX_DONE");
      break;

    case NRF_SERIAL_EVENT_RX_DATA:
      NRF_LOG_DEBUG("NRF_SERIAL_EVENT_RX_DATA");
      serial_receive_data(p_serial);
      break;

    case NRF_SERIAL_EVENT_DRV_ERR:
      NRF_LOG_ERROR("NRF_SERIAL_EVENT_DRV_ERR(0x%08x)", p_serial->p_ctx->error);
      // TO-DO reset UART RX by GPIO
      break;

    case NRF_SERIAL_EVENT_FIFO_ERR:
      NRF_LOG_ERROR("NRF_SERIAL_EVENT_FIFO_ERR");
      // TO-DO reset UART RX by GPIO
      break;

    default:
      break;
  }
}

uint32_t inter_connect_init(data_handler_t data_handler)
{
  if (data_handler == NULL)
  {
    NRF_LOG_ERROR("Data handler is null");
		return NRF_ERROR_INVALID_PARAM;
  }
  else
  {
    data_handler_cb = data_handler;
  }

  uart_rx_init();

  return nrf_serial_init(&m_serial_uart, &uart0_drv_config, &serial_config);
}

/**@brief Function for un-initialization */
uint32_t inter_connect_uninit(void)
{
  return nrf_serial_uninit(&m_serial_uart);
}

uint32_t inter_connect_send(uint8_t data_type, const uint8_t *data_buff, uint8_t data_len)
{
  uint8_t bcc = 0x00;
  uint32_t ret_code;
  
  if (data_len > (DEV_CTRL_MAX_DATA_LEN - PROT_OVEREAD))
  {
    return NRF_ERROR_INVALID_PARAM;
  }

  // Construct full data array  
  m_tx_buf.length             = data_len + PROT_OVEREAD;
  m_tx_buf.data[PROT_STX_POS] = PROT_HEADER_STX;
  m_tx_buf.data[PROT_TYP_POS] = data_type;
  m_tx_buf.data[PROT_LEN_POS] = data_len;
  if (data_buff != NULL)
  {
    memcpy(&m_tx_buf.data[PROT_LEN_POS+1], data_buff, data_len);
  }
#if APP_CFG_ENABLE_BCC
  for (uint32_t i = 1; i < (m_tx_buf.length-1); i++)
  {
    bcc ^= data_array[i];
  }
#endif
  m_tx_buf.data[PROT_LEN_POS+data_len+1] = bcc;
 
  NRF_LOG_INFO("UART SEND, length:%d", m_tx_buf.length);
  NRF_LOG_HEXDUMP_INFO(m_tx_buf.data, m_tx_buf.length);
  
  ret_code = nrf_serial_write(&m_serial_uart, m_tx_buf.data, m_tx_buf.length, NULL, 0);
  if (ret_code != NRF_SUCCESS)
  {
    return ret_code;
  }

  return nrf_serial_flush(&m_serial_uart, 0);
	
}
