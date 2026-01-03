#ifndef _HAL_SWITCH_H_
#define _HAL_SWITCH_H_

void set_adc_ch(uint8_t adc_ch);

void stop_adc_ch(uint8_t adc_ch);

void set_channel_pin(uint8_t ch, GPIO_PinState pin_status);

#endif /* _HAL_SWITCH_H_ */
