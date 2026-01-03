#include "user_comm.h"

uint8_t bl_tx_buf[OLD_FRAME_LEN] = {0};
uint8_t AT_FB[] = {'+', 'R', 'E', 'A', 'D', 'Y', '\r', '\n'};
uint8_t CONNECTED_AT_FB[] = {
    '+',
    'C',
    'O',
    'N',
    'N',
    'E',
};
uint8_t DIS_CONN_AT_FB[] = {'+', 'D', 'I', 'S', 'C', 'O', 'N'};
uint8_t bl_conn_status = 0;

// 40ms传输一帧数据
void bl_task(void)
{
    static uint32_t last_send_tick = 0;

    // 调节帧率
    if (HAL_GetTick() - last_send_tick < 40)
        return;

    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)bl_tx_buf, OLD_FRAME_LEN);
    // HAL_UART_Transmit(&huart2, (uint8_t *)bl_tx_buf, OLD_FRAME_LEN, 0xffff);

    last_send_tick = HAL_GetTick();
}

void conn_status_check(void)
{
    if (memcmp(uart2_rx_buf, AT_FB, sizeof(AT_FB)) == 0)
    {
        // bl_conn_status = 1;
    }
    else if (memcmp(uart2_rx_buf, CONNECTED_AT_FB, sizeof(CONNECTED_AT_FB)) == 0)
    {
        bl_conn_status = 1;
        // all_led_off();
    }
    else if (memcmp(uart2_rx_buf, DIS_CONN_AT_FB, sizeof(DIS_CONN_AT_FB)) == 0)
    {

        bl_conn_status = 0;
        // all_led_off();
    }
}

void recv_handler_u2(uint8_t len)
{
    // 如果已经连接上了, 就是透传模式, 理论上, 从串口发送过来的数据数据量不大.
}

void u2_task(void)
{

    if (got_rx_u2)
    {

        bl_cmd_handler(uart2_rx_buf, got_rx_u2);
        got_rx_u2 = 0;
    }
}

void bl_link_status_check(void)
{
    static uint32_t last_run_ts = 0;
    if (HAL_GetTick() - last_run_ts < 1000)
    {
        return;
    }
    last_run_ts = HAL_GetTick();

    // 读取bl link引脚
    if (HAL_GPIO_ReadPin(BL_LINK_GPIO_Port, BL_LINK_Pin) == GPIO_PIN_SET)
    {
        if (bl_conn_status == 0)
        {
            DBG_PRINTF("BL Link connected\r\n");
        }
        bl_conn_status = 1;
    }
    else
    {
        if (bl_conn_status)
        {
            DBG_PRINTF("BL Link disconnected\r\n");
        }
        bl_conn_status = 0;
    }
}

