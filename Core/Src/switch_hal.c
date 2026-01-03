#include "user_comm.h"

const GPIO_Channel channels[64] = {CH_DEF(0), CH_DEF(1), CH_DEF(2), CH_DEF(3),
                                   CH_DEF(4), CH_DEF(5), CH_DEF(6), CH_DEF(7),
                                   CH_DEF(8), CH_DEF(9), CH_DEF(10), CH_DEF(11),
                                   CH_DEF(12), CH_DEF(13)};

void set_channel_pin(uint8_t ch, GPIO_PinState pin_status)
{
    if (ch < sizeof(channels) / sizeof(channels[0]))
    {
        HAL_GPIO_WritePin(channels[ch].port, channels[ch].pin, pin_status);
    }
}
const uint8_t adc_ch_mapping[16] = {15, 14, 13, 12,
                                    11, 10, 9, 8,
                                    0, 1, 2, 3,
                                    4, 5, 6, 7};
void set_adc_ch(uint8_t adc_ch)
{

    if (adc_ch < 16)
    {
        uint8_t mapped_ch = adc_ch_mapping[adc_ch];

        HAL_GPIO_WritePin(SW_4067_S0_GPIO_Port, SW_4067_S0_Pin, (mapped_ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SW_4067_S1_GPIO_Port, SW_4067_S1_Pin, (mapped_ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SW_4067_S2_GPIO_Port, SW_4067_S2_Pin, (mapped_ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SW_4067_S3_GPIO_Port, SW_4067_S3_Pin, (mapped_ch & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        HAL_GPIO_WritePin(SW_4067_EN_1_GPIO_Port, SW_4067_EN_1_Pin, GPIO_PIN_RESET);
    }
    else if (adc_ch < 20)
    {
        adc_ch -= 16;

        HAL_GPIO_WritePin(SW_4067_S0_GPIO_Port, SW_4067_S0_Pin, (adc_ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SW_4067_S1_GPIO_Port, SW_4067_S1_Pin, (adc_ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SW_4067_S2_GPIO_Port, SW_4067_S2_Pin, (adc_ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SW_4067_S3_GPIO_Port, SW_4067_S3_Pin, (adc_ch & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        HAL_GPIO_WritePin(SW_4067_EN_2_GPIO_Port, SW_4067_EN_2_Pin, GPIO_PIN_RESET);
    }
}

void stop_adc_ch(uint8_t adc_ch)
{

    if (adc_ch < 16)
    {
        HAL_GPIO_WritePin(SW_4067_EN_1_GPIO_Port, SW_4067_EN_1_Pin, GPIO_PIN_SET);
    }
    else if (adc_ch < 20)
    {
        HAL_GPIO_WritePin(SW_4067_EN_2_GPIO_Port, SW_4067_EN_2_Pin, GPIO_PIN_SET);
    }
}