#ifndef _SYS_H_
#define _SYS_H_

void delay_init(void);

void delay_ms(uint32_t ms);

void delay_us(uint32_t nus);

uint16_t ema_u16(uint16_t new_data, uint16_t last_data, uint16_t a_num, uint16_t a_den);

void feed_iwdg(void);

typedef enum
{
    PRESS_ROW = 0,
    PRESS_COL = 1
} press_type_t;

void press256(uint8_t *in, uint8_t *out, uint16_t width, uint16_t height, uint16_t value, press_type_t type);

#endif /* _SYS_H_ */

