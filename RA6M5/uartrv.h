/*
 * uartrv.h
 *
 *  Created on: 2026年7月5日
 *      Author: Riloptop
 */

#ifndef DEBUG_UARTRV_UARTRV_H_
#define DEBUG_UARTRV_UARTRV_H_

#include "hal_data.h"
#include <stdint.h>
#include <stdbool.h>

#define UART2_RX_RING_SIZE      512
#define UART2_LINE_BUF_SIZE     256
#define UART2_TX_TIMEOUT_MS     1000

fsp_err_t uart2_init(void);

fsp_err_t uart2_send(const uint8_t *data, uint32_t len);
fsp_err_t uart2_send_string(const char *str);

bool uart2_get_byte(uint8_t *data);
uint16_t uart2_available(void);
void uart2_clear_rx_buffer(void);

/*
 * 获取一包不定长字符串数据
 * 默认以 '\n' 作为一包结束
 */
bool uart2_get_line(uint8_t *line_buf, uint16_t max_len, uint16_t *out_len);

/*
 * FSP Configurator 里的 UART callback 名字填这个
 */
void g_uart2_callback(uart_callback_args_t *p_args);
void uart2_handle_packet(uint8_t *data, uint16_t len);




#endif /* DEBUG_UARTRV_UARTRV_H_ */
