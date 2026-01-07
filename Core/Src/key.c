#include "user_comm.h"

uint8_t imu_reseted = 0;

static uint32_t key_down_counter = 0;

// 10ms执行一次
void key_task(void)
{
    static uint32_t last_run_tck = 0;
    uint32_t now = HAL_GetTick();

    // 调节帧率
    if (now - last_run_tck < 10)
        return;

    if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
    {
        // KEY1按下
        key_down_counter++;

        // 6秒关机
        if (key_down_counter >= POWER_DOWN_COUNTER)
        {

            // 关灯
            all_led_off();

            sys_shutdown();
        }
    }
    else
    {
        if (key_down_counter >= RESET_GQ_KEY_SHAKE_DELAY) // 1s
        {
            imu_reseted = 1;
            qst_vqf_init(0.01f);
        }
        key_down_counter = 0;
    }

    last_run_tck = now;
}

void sys_shutdown(void)
{
    while (1)
    {

        // 喂狗
        feed_iwdg();

        // 拉低电源控制引脚
        HAL_GPIO_WritePin(PWR_CTRL_GPIO_Port, PWR_CTRL_Pin, GPIO_PIN_RESET);
        // 等待关机`
    }
}