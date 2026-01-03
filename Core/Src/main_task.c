#include "user_comm.h"

uint8_t main_adc_buf[ADC1_BUF_SIZE];
volatile uint8_t points_data[FRAME_LEN] = {0};
volatile uint8_t need_send_reset_signal = 0;
volatile uint8_t imu_rest_tx_data[5] = {0x03, 0xAA, 0x55, 0x03, 0x99};
volatile uint8_t tx_data[OLD_FRAME_LEN] = {0};
float q_out[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // 初始四元数
volatile uint8_t main_adc_busy = 0;
volatile uint8_t bat_adc_done = 0;

void adc_data_handler_with_idx(uint8_t point_nmb)
{
    // 简单计算平均值
    float adc_sum = 0;
    uint32_t i = 0;

    for (i = 0; i < ADC1_BUF_SIZE; i++)
    {
        adc_sum += (float)main_adc_buf[i];
    }
    float result = adc_sum / ADC1_BUF_SIZE;
    points_data[point_nmb] = (uint8_t)result; // 将结果存储到points_data中
    // points_data[point_nmb] = point_nmb;
}

void uart_send(void)
{
    HAL_StatusTypeDef status = HAL_OK;
    // 先发送复位信号
    if (need_send_reset_signal)
    {
        status = HAL_UART_Transmit(&huart1, (uint8_t *)imu_rest_tx_data, 5, 0xffff);
        need_send_reset_signal = 0;
    }

#if USE_PRESS
    memcpy(&tx_data[2], (const void *)points_data_after_proc, TOTAL_POINTS / 2);
    memcpy(&tx_data[TOTAL_POINTS / 2 + 8], (const void *)&points_data_after_proc[TOTAL_POINTS / 2], TOTAL_POINTS / 2);
#else
    memcpy(&tx_data[2], (const void *)points_data, TOTAL_POINTS / 2);
    memcpy(&tx_data[TOTAL_POINTS / 2 + 8], (const void *)&points_data[TOTAL_POINTS / 2], TOTAL_POINTS / 2);
#endif

    // 4元数放在最后16个字节
    memcpy(&tx_data[TOTAL_POINTS + 8], (const void *)q_out, 16);

    // 将数据复制到bl的传输数组中
    memcpy(bl_tx_buf, (const void *)tx_data, OLD_FRAME_LEN);

    status = HAL_UART_Transmit_DMA(&huart1, (uint8_t *)tx_data, OLD_FRAME_LEN);
}

void main_task(void)
{
    uint16_t input_idx = 0;
    uint16_t adc_idx = 0;
    uint16_t point_nmb = 0;
    uint32_t start_ms = HAL_GetTick();

    for (input_idx = 0; input_idx < INPUT_CH_NUMBER; input_idx++)
    {
        // 打开gpio输出
        set_channel_pin(input_idx, GPIO_PIN_SET);

        // 读取对应的ADC值
        for (adc_idx = 0; adc_idx < ADC_CH_NUMBER; adc_idx++)
        {
            
            point_nmb = input_idx * ADC_CH_NUMBER + adc_idx;

            set_adc_ch(adc_idx);

            delay_us(10); // 稳定时间

            adc_data_handler_with_idx(point_nmb);

            stop_adc_ch(adc_idx);
        }

        // 关闭gpio输出
        set_channel_pin(input_idx, GPIO_PIN_RESET);
    }

    HAL_UART_Transmit_DMA(&huart1, points_data, FRAME_LEN);
    // uart_send();
    uint32_t end_ms = HAL_GetTick();

    // about 4ms
    // DBG_PRINTF("ADC and UART send time: %lu ms\r\n", end_ms - start_ms);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        main_adc_busy = 0;
    }
    else if (hadc->Instance == ADC2)
    {
        bat_adc_done = 1;
    }
}