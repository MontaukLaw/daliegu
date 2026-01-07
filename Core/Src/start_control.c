#include "user_comm.h"

// 长按2秒开机
void start_control(void)
{

    uint16_t i = 0;
    // delay2秒开机
    delay_ms(2000);

    // 开机了.
    HAL_GPIO_WritePin(PWR_CTRL_GPIO_Port, PWR_CTRL_Pin, GPIO_PIN_SET);

    // 开机, 开绿灯
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    // delay_ms(1000);
}