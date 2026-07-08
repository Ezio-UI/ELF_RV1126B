#include "uart.h"


void uart7_Init(void)
{
    fsp_err_t err = FSP_SUCCESS;
    err = R_SCI_UART_Open(&uart7_ctrl, &uart7_cfg);
    assert(FSP_SUCCESS == err);
}

volatile bool uart_send_complete_flag = false;

void uart7_callback(uart_callback_args_t *p_args)
{
    switch(p_args->event)
    {
        case UART_EVENT_RX_CHAR:
        {
            R_SCI_UART_Write(&uart7_ctrl, (uint8_t *)&(p_args->data), 1);
            break;
        }
        case UART_EVENT_TX_COMPLETE:
        {
           uart_send_complete_flag = true;
            break;
        }
        default:
        break;
    }
}

/*重定向printf输出*/
#if defined __GNUC__&& !defined __clang__
int _write(int fd,char *pBuffer, int size);
int _write(int fd,char *pBuffer, int size)
{
    (void)fd;
    R_SCI_UART_Write(&uart7_ctrl, (uint8_t *)pBuffer,(uint32_t)size);
    while (uart_send_complete_flag == false);
    uart_send_complete_flag = false;


    return size;
}
#else
int fputc(int ch,FILE *f)
{
    (void)f;
    R_SCI_UART_Write(&uart7_ctrl, (uint8_t *)&ch,1);while (uart_send_complete_flag == false);
    uart_send_complete_flag = false;

    return ch;
}
#endif


