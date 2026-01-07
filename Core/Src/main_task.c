#include "user_comm.h"

uint8_t main_adc_buf[ADC1_BUF_SIZE];
volatile uint8_t points_data[TOTAL_POINTS] = {0};
volatile uint8_t points_data_1024[1024] = {0};
volatile uint8_t need_send_reset_signal = 0;
volatile uint8_t imu_rest_tx_data[5] = {0x03, 0xAA, 0x55, 0x03, 0x99};
volatile uint8_t tx_data[OLD_FRAME_LEN] = {0};

volatile uint8_t main_adc_busy = 0;
volatile uint8_t bat_adc_done = 0;
volatile uint8_t check_reset_imu = 0;
volatile uint8_t uart_busy = 0; // UART是否忙碌
volatile uint8_t bl_uart_tx_done = 0;

uint8_t mems_data[16] = {0xF1, 0xFF, 0x7F, 0x3F, 0xE0, 0xB6, 0x2F, 0x3A,
                         0xF0, 0xF0, 0x08, 0xBA, 0x88, 0x7C, 0x84, 0xBA};

void adc_data_handler_with_idx(uint16_t point_nmb, uint8_t *points_data)
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

// 根据协议修改一下.
void fill_tx_data(void)
{
    // 第一个包
    tx_data[0] = 0xaa;
    tx_data[1] = 0x55;
    tx_data[2] = 0x03;
    tx_data[3] = 0x99;
    tx_data[4] = FIRST_PACK_IDX;
    tx_data[5] = LEFT_HAND;

    // 第二个包
    tx_data[FIRST_PACK_START_IDX + TOTAL_POINTS / 2] = 0xaa;
    tx_data[FIRST_PACK_START_IDX + TOTAL_POINTS / 2 + 1] = 0x55;
    tx_data[FIRST_PACK_START_IDX + TOTAL_POINTS / 2 + 2] = 0x03;
    tx_data[FIRST_PACK_START_IDX + TOTAL_POINTS / 2 + 3] = 0x99;
    tx_data[FIRST_PACK_START_IDX + TOTAL_POINTS / 2 + 4] = SECOND_PACK_IDX;
    tx_data[FIRST_PACK_START_IDX + TOTAL_POINTS / 2 + 5] = LEFT_HAND;
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
    // memcpy(&tx_data[2], (const void *)points_data_after_proc, TOTAL_POINTS / 2);
    // memcpy(&tx_data[TOTAL_POINTS / 2 + 8], (const void *)&points_data_after_proc[TOTAL_POINTS / 2], TOTAL_POINTS / 2);
#else

    // 将一半数据复制到tx_data
    memcpy(&tx_data[FIRST_PACK_START_IDX], (const void *)points_data, TOTAL_POINTS / 2);

    // 另一半数据复制到tx_data
    memcpy(&tx_data[SECOND_PACK_START_IDX], (const void *)&points_data[TOTAL_POINTS / 2], TOTAL_POINTS / 2);

#endif

    // 4元数放在最后16个字节
    memcpy(&tx_data[TOTAL_POINTS + PACK_TYPE_LEN * 2 + PACK_IDX_LEN * 2 + PACKAGE_HEAD_LEN * 2], (const void *)q_out, sizeof(float) * 4);

    // 将数据复制到bl的传输数组中
    memcpy(bl_tx_buf, (const void *)tx_data, OLD_FRAME_LEN);

    status = HAL_UART_Transmit_DMA(&huart1, (uint8_t *)tx_data, OLD_FRAME_LEN);
}

// Main 10 times main task cost: 55 ms
void main_task(void)
{
    uint16_t input_idx = 0;
    uint16_t adc_idx = 0;
    uint16_t point_nmb = 0;
    for (input_idx = 0; input_idx < INPUT_CH_NUMBER; input_idx++)
    {

        set_channel_pin(input_idx, GPIO_PIN_SET);

        // 读取对应的ADC值
        for (adc_idx = 0; adc_idx < ADC_CH_NUMBER; adc_idx++)
        {
            point_nmb = input_idx * INPUT_CH_NUMBER + adc_idx;

            set_adc_ch(adc_idx);
            delay_us(10); // 稳定时间

            adc_data_handler_with_idx(point_nmb, points_data);

            stop_adc_ch(adc_idx);
        }

        // 关闭gpio输出
        set_channel_pin(input_idx, GPIO_PIN_RESET);
    }

    uart_send();
}

void main_task_1024(void)
{
    uint16_t input_idx = 0;
    uint16_t adc_idx = 0;
    uint16_t point_nmb = 0;
    uint32_t start_ms = HAL_GetTick();

    memset((void *)points_data_1024, 0, 1024);

    for (input_idx = 0; input_idx < 32; input_idx++)
    {
        if (input_idx < INPUT_CH_NUMBER)
        {
            // 打开gpio输出
            set_channel_pin(input_idx, GPIO_PIN_SET);
        }

        // 读取对应的ADC值
        for (adc_idx = 0; adc_idx < 32; adc_idx++)
        {
            if (input_idx < ADC_CH_NUMBER)
            {

                point_nmb = input_idx * 32 + adc_idx;

                set_adc_ch(adc_idx);

                delay_us(10); // 稳定时间

                adc_data_handler_with_idx(point_nmb, points_data_1024);

                stop_adc_ch(adc_idx);
            }
        }

        if (input_idx < INPUT_CH_NUMBER)
        {
            // 关闭gpio输出
            set_channel_pin(input_idx, GPIO_PIN_RESET);
        }
    }

    const uint8_t head_info[4] = {0xaa, 0x55, 0x03, 0x99};
    HAL_UART_Transmit(&huart1, (uint8_t *)head_info, 4, 0xFFFF);

    // HAL_UART_Transmit(&huart1, (uint8_t *)points_data, TOTAL_POINTS, 0xFFFF);

    // const uint8_t rest_zeros[1024 - TOTAL_POINTS] = {0};
    // HAL_UART_Transmit(&huart1, (uint8_t *)rest_zeros, 1024 - TOTAL_POINTS, 0xFFFF);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)points_data_1024, 1024);

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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart_busy = 0; // UART发送完成

        check_reset_imu = 1;
    }
    else if (huart->Instance == USART2)
    {
        // 蓝牙发送完成
        bl_uart_tx_done = 1;
    }
}

void imu_rest_cmd_task(void)
{
    static uint8_t imu_reseted_sent = 0;
    if (check_reset_imu)
    {
        if (imu_reseted && imu_reseted_sent == 0)
        {

            need_send_reset_signal = 1;

            imu_reseted_sent = 1;
        }

        check_reset_imu = 0;
    }

    if (bl_uart_tx_done)
    {
        if (imu_reseted && imu_reseted_sent)
        {

            // HAL_UART_Transmit_DMA(&huart2, (uint8_t *)imu_rest_tx_data, 5);
            HAL_UART_Transmit(&huart2, (uint8_t *)imu_rest_tx_data, 5, 0xffff);

            imu_reseted_sent = 0;
            imu_reseted = 0;
        }

        bl_uart_tx_done = 0;
    }
}
