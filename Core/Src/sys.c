#include "user_comm.h"

static uint8_t fac_us = 0; // us延时倍乘数

void delay_init(void)
{
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK); // SysTick频率为HCLK
    fac_us = 170;                                        // 不论是否使用OS,fac_us都需要使用
}

void delay_ms(uint32_t ms)
{
    uint32_t i;
    for (i = 0; i < ms; i++)
    {
        delay_us(1000);

#if IWDG_ENABLED
        HAL_IWDG_Refresh(&hiwdg);
#endif
    }
}

void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD; // LOAD的值
    ticks = nus * fac_us;            // 需要的节拍数
    told = SysTick->VAL;             // 刚进入时的计数器值
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow; // 这里注意一下SYSTICK是一个递减的计数器就可以了.
            else
                tcnt += reload - tnow + told;
            told = tnow;
            if (tcnt >= ticks)
                break; // 时间超过/等于要延迟的时间,则退出.
        }
    };
}

uint16_t ema_u16(uint16_t new_data, uint16_t last_data, uint16_t a_num, uint16_t a_den)
{
    return (uint16_t)((a_num * new_data + (a_den - a_num) * last_data) / a_den);
}

void feed_iwdg(void)
{
#if ENABLE_IWDG
    HAL_IWDG_Refresh(&hiwdg);
#endif
}

void press256(uint8_t *in, uint8_t *out, uint16_t width, uint16_t height, uint16_t value, press_type_t type)
{
    uint16_t i, j;

    memcpy(out, in, width * height * sizeof(uint8_t));

    if (type == PRESS_ROW)
    {
        // 每一行的和
        uint32_t rowSum[height];

        for (i = 0; i < height; i++)
        {
            uint32_t sum = 0;
            for (j = 0; j < width; j++)
            {
                sum += out[i * width + j];
            }
            rowSum[i] = sum;
        }

        // 压缩
        for (i = 0; i < height; i++)
        {
            for (j = 0; j < width; j++)
            {
                uint16_t idx = i * width + j;
                uint16_t oldVal = out[idx];

                out[idx] = oldVal +
                           (uint16_t)((rowSum[i] - oldVal) / value);
            }
        }
    }
    else // PRESS_COL
    {
        // 每一列的和
        uint32_t colSum[height];

        for (i = 0; i < height; i++)
        {
            uint32_t sum = 0;
            for (j = 0; j < width; j++)
            {
                sum += out[j * height + i];
            }
            colSum[i] = sum;
        }

        // 压缩
        for (i = 0; i < height; i++)
        {
            for (j = 0; j < width; j++)
            {
                uint16_t idx = j * height + i;
                uint16_t oldVal = out[idx];

                if (oldVal < 6)
                    continue;

                out[idx] = oldVal +
                           (uint16_t)((colSum[i] - oldVal) / value);
            }
        }
    }
}
