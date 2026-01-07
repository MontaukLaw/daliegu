#ifndef _MAIN_TASK_H_
#define _MAIN_TASK_H_

void main_task(void);

void fill_tx_data(void);

void imu_rest_cmd_task(void);

extern uint8_t main_adc_buf[];

extern volatile uint8_t bat_adc_done;

#endif /* _MAIN_TASK_H_ */
