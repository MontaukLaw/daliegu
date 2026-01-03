#ifndef _USER_COMM_H_
#define _USER_COMM_H_

#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "sys.h"

#include "iwdg.h"
#include "switch_hal.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "main_task.h"
#include "bat_val.h"
#include "adc.h"
#include "usart.h"
#include "bl.h"
#include <string.h>
#include "comm.h"

#define DEBUG_PRINT 1

#if DEBUG_PRINT
#define DBG_PRINTF(fmt, ...) \
    printf("[%s:%d] " fmt, __func__, __LINE__, ##__VA_ARGS__)
#else
#define DBG_PRINTF(...) ((void)0)
#endif

#define CH_DEF(n) {OUT_##n##_GPIO_Port, OUT_##n##_Pin}

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} GPIO_Channel;

#define IWDG_ENABLED 1

#define INPUT_CH_NUMBER 14
#define ADC_CH_NUMBER 20

#define ADC2_BUF_SIZE 500

#define ADC1_BUF_SIZE 10

#define TOTAL_POINTS (INPUT_CH_NUMBER * ADC_CH_NUMBER)
#define FRAME_LEN (TOTAL_POINTS + 4)

#define OLD_FRAME_LEN (TOTAL_POINTS + 8 + 4 + 16)

#define STANDARD_PROTOCAL_LEN 64
#define UART_RX_BUF_LEN STANDARD_PROTOCAL_LEN

// ??????
#define CMD_GET_DATA 0x01

#define CMD_TYPE_REQUEST 0x00
#define CMD_TYPE_RESPONSE 0x01
#define CMD_TYPE_NOTIFICATION 0x02

#define CMD_TOTAL_LEN 0x08

#define CMD_RESULT_SUCCESS 0x00
#define CMD_RESULT_FAIL 0x01

#define ADC2_DMA_BUF_LEN 100

#endif /* _USER_COMM_H_ */
