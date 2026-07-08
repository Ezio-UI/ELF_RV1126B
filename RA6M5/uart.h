#ifndef UART_H_
#define UART_H_
#include "hal_data.h"
#include "stdio.h"

void uart7_Init(void);
void uart7_callback(uart_callback_args_t *p_args);

#endif
