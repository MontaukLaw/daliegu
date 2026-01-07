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
#include "key.h"
#include "charge_state.h"
#include "check_mems_type.h"
#include "icm42688.h"
#include "icm42688_hard_i2c.h"
#include "mems.h"
#include "qst_ahrs.h"
#include "g_sensor.h"
#include "qmi8658_app.h"
#include "led.h"
#include "app_internal_flash.h"
#include "i2c.h"
#include <math.h>

#define ENABLE_IWDG 1

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

#define PACKAGE_HEAD_LEN 4
#define PACK_IDX_LEN 1
#define PACK_TYPE_LEN 1
#define TOTAL_POINTS (INPUT_CH_NUMBER * ADC_CH_NUMBER)
#define FRAME_LEN (TOTAL_POINTS + 4)

// 旧版本的帧长度
// 1. 包头 4字节*2
// 2. 包索引 1字节*2
// 3. 包类型 1字节*2
// 4. 数据区 TOTAL_POINTS 字节
// 5. 四元数 16字节
#define OLD_FRAME_LEN (TOTAL_POINTS + PACKAGE_HEAD_LEN + PACKAGE_HEAD_LEN + PACK_IDX_LEN + PACK_IDX_LEN + PACK_TYPE_LEN + PACK_TYPE_LEN + sizeof(float) * 4)

#define PACKAGE_1_LEN TOTAL_POINTS / 2
#define PACKAGE_2_LEN TOTAL_POINTS / 2 + 16

#define STANDARD_PROTOCAL_LEN 64
#define UART_RX_BUF_LEN STANDARD_PROTOCAL_LEN

#define CMD_GET_DATA 0x01

#define CMD_TYPE_REQUEST 0x00
#define CMD_TYPE_RESPONSE 0x01
#define CMD_TYPE_NOTIFICATION 0x02

#define CMD_TOTAL_LEN 0x08

#define CMD_RESULT_SUCCESS 0x00
#define CMD_RESULT_FAIL 0x01

#define ADC2_DMA_BUF_LEN 100

#define LEFT_HAND 0x01
#define RIGHT_HAND 0x02

#define HAND_DIRECT LEFT_HAND
#define FIRST_PACK_IDX 1
#define SECOND_PACK_IDX 2

#define FIRST_PACK_START_IDX 6
#define SECOND_PACK_START_IDX (FIRST_PACK_START_IDX + PACKAGE_1_LEN + 4 + 2)

// 6秒关灯
#define POWER_DOWN_COUNTER 350 // 600

#define RESET_GQ_KEY_SHAKE_DELAY 5

///////////////// CHARGE STATE ////////////////////
#define CHARGING_FINISHED 1
#define CHARGING_ONGOING 2
#define CHARGING_NO_POWER 0

///////////////// MEMS ///////////////////
#define MEMS_TYPE_ICM42688 1
#define MEMS_TYPE_QMI8658 2
#define MEMS_TYPE_UNKNOWN 0
#define DEG2RAD 0.017453293f /* 度转弧度 π/180 */
#define MAX_STATIC_CALI_COUNTER 100


#endif /* _USER_COMM_H_ */
