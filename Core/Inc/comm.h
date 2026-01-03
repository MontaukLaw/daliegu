#ifndef _COMM_H_
#define _COMM_H_

void start_uart_rx(void);

void com_task(void);

void uart1_it_handler(void);

void uart2_it_handler(void);

void bl_cmd_handler(uint8_t *data, uint8_t len);

extern uint8_t got_rx_u2;
extern uint8_t uart2_rx_buf[];

#endif // _COMM_H_
