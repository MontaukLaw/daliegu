#ifndef _G_SENSOR_H_
#define _G_SENSOR_H_

void gsensor_task(void);

void qmi8658_task(void);

void reset_q_out(void);

uint8_t atk_qmi8658_check_whoami(void);

#endif

