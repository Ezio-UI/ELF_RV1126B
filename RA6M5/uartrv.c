/*
 * uartrv.c
 *
 *  Created on: 2026年7月5日
 *      Author: Riloptop
 */

#include "hal_data.h"
#include "uartrv.h"
#include "stdbool.h"
#include <string.h>
#include "debug_electrical_stimulation/electrical_stimulation.h"

static volatile bool g_uart2_opened  = false;
static volatile bool g_uart2_tx_done = true;

static volatile uint8_t  g_uart2_rx_ring[UART2_RX_RING_SIZE];
static volatile uint16_t g_uart2_rx_write = 0;
static volatile uint16_t g_uart2_rx_read  = 0;
static volatile uint8_t  g_uart2_rx_overflow = 0;

/* callback 里调用：只存数据，不做复杂处理 */
static void uart2_rx_push_from_isr(uint8_t data)
{
    uint16_t next = (uint16_t)((g_uart2_rx_write + 1) % UART2_RX_RING_SIZE);

    if (next != g_uart2_rx_read)
    {
        g_uart2_rx_ring[g_uart2_rx_write] = data;
        g_uart2_rx_write = next;
    }
    else
    {
        g_uart2_rx_overflow = 1;
    }
}

fsp_err_t uart2_init(void)
{
    fsp_err_t err;

    if (g_uart2_opened)
    {
        return FSP_SUCCESS;
    }

    err = g_uart2.p_api->open(g_uart2.p_ctrl, g_uart2.p_cfg);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    g_uart2_tx_done = true;
    g_uart2_opened  = true;

    return FSP_SUCCESS;
}

fsp_err_t uart2_send(const uint8_t *data, uint32_t len)
{
    fsp_err_t err;
    uint32_t timeout = 0;

    if ((data == NULL) || (len == 0))
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    g_uart2_tx_done = false;

    err = g_uart2.p_api->write(g_uart2.p_ctrl, data, len);
    if (FSP_SUCCESS != err)
    {
        g_uart2_tx_done = true;
        return err;
    }

    while (!g_uart2_tx_done)
    {
        R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);

        timeout++;
        if (timeout >= UART2_TX_TIMEOUT_MS)
        {
            g_uart2_tx_done = true;
            return FSP_ERR_TIMEOUT;
        }
    }

    return FSP_SUCCESS;
}

fsp_err_t uart2_send_string(const char *str)
{
    if (str == NULL)
    {
        return FSP_ERR_INVALID_ARGUMENT;
    }

    return uart2_send((const uint8_t *)str, (uint32_t)strlen(str));
}

bool uart2_get_byte(uint8_t *data)
{
    if (data == NULL)
    {
        return false;
    }

    __disable_irq();

    if (g_uart2_rx_read == g_uart2_rx_write)
    {
        __enable_irq();
        return false;
    }

    *data = g_uart2_rx_ring[g_uart2_rx_read];
    g_uart2_rx_read = (uint16_t)((g_uart2_rx_read + 1) % UART2_RX_RING_SIZE);

    __enable_irq();

    return true;
}

uint16_t uart2_available(void)
{
    uint16_t count;

    __disable_irq();

    if (g_uart2_rx_write >= g_uart2_rx_read)
    {
        count = (uint16_t)(g_uart2_rx_write - g_uart2_rx_read);
    }
    else
    {
        count = (uint16_t)(UART2_RX_RING_SIZE - g_uart2_rx_read + g_uart2_rx_write);
    }

    __enable_irq();

    return count;
}

void uart2_clear_rx_buffer(void)
{
    __disable_irq();

    g_uart2_rx_write = 0;
    g_uart2_rx_read  = 0;
    g_uart2_rx_overflow = 0;

    __enable_irq();
}

/*
 * 从环形缓冲区里拼一包字符串数据
 * RV1126B 发送数据时，最后加 '\n'
 *
 * 例如：
 *     PING\n
 *     DIST:120\n
 *     PERSON,80\n
 */
bool uart2_get_line(uint8_t *line_buf, uint16_t max_len, uint16_t *out_len)
{
    static uint8_t  packet_buf[UART2_LINE_BUF_SIZE];
    static uint16_t packet_len = 0;

    uint8_t ch;

    if ((line_buf == NULL) || (max_len == 0))
    {
        return false;
    }

    while (uart2_get_byte(&ch))
    {
        if (ch == '\r')
        {
            continue;
        }

        if (ch == '\n')
        {
            uint16_t copy_len = packet_len;

            if (copy_len >= max_len)
            {
                copy_len = (uint16_t)(max_len - 1);
            }

            memcpy(line_buf, packet_buf, copy_len);
            line_buf[copy_len] = '\0';

            if (out_len != NULL)
            {
                *out_len = copy_len;
            }

            packet_len = 0;

            return true;
        }
        else
        {
            if (packet_len < UART2_LINE_BUF_SIZE - 1)
            {
                packet_buf[packet_len++] = ch;
            }
            else
            {
                /*
                 * 当前包太长，丢弃这一包
                 */
                packet_len = 0;
            }
        }
    }

    return false;
}

void g_uart2_callback(uart_callback_args_t *p_args)
{
    if (p_args == NULL)
    {
        return;
    }

    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
        {
            uint8_t ch = (uint8_t)p_args->data;
            uart2_rx_push_from_isr(ch);
            break;
        }

        case UART_EVENT_TX_COMPLETE:
        {
            g_uart2_tx_done = true;
            break;
        }

        case UART_EVENT_ERR_PARITY:
        case UART_EVENT_ERR_FRAMING:
        case UART_EVENT_ERR_OVERFLOW:
        {
            g_uart2_rx_overflow = 1;
            break;
        }

        default:
        {
            break;
        }
    }
}

void uart2_handle_packet(uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0))
    {
        return;
    }
    char buf[128] = {0};
    memcpy(buf, data, len);
    /*
     * 示例 1：RV1126B 发送 PING\n
     * 瑞萨回复 PONG\n
     */
    if (strcmp((char *)data, "PING") == 0)
    {
        uart2_send_string("PONG\n");
        return;
    }

    /* 障碍物数据解析 DIR:xx,DIST:xxx 单位cm */
       if (strstr(buf, "DIR:") != NULL && strstr(buf, "DIST:") != NULL)
       {
           char dir_str[3] = {0};
           char dist_str[5] = {0};
           uint16_t distance_cm;
           uint8_t dir;

           // 提取方向
           char *p_dir = strstr(buf, "DIR:") + 4;
           strncpy(dir_str, p_dir, 2);
           dir = atoi(dir_str);

           // 提取距离(cm)
           char *p_dist = strstr(buf, "DIST:") + 5;
           strncpy(dist_str, p_dist, 4);
           distance_cm = atoi(dist_str);


           electrical_stimulation_freCtrl(dir,distance_cm);
           return;
       }

}
