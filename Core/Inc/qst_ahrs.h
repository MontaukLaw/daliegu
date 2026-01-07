#ifndef QST_AHRS_H
#define QST_AHRS_H

#include <stdint.h>
#include <stdio.h>

/**
 * @brief : algorithm initial function.
 * @param[in] : none
 * @param[out] : none
 * @return: 1
 */
int qst_vqf_init(float dt);

/**
 * @brief : algorithm entry
 * @param[in] : float fusion_accel[3]        acc data  m/s/s    9.8 = 1g  50HZ  direction (right-x, front-y, up-z)
 * @param[in] : float fusion_gyro[3]         gyro data rad/s    50HZ
 * @param[in] : float *fusion_dt             The scheduling time of the algorithm library, in seconds
 * @param[out] : float rpy[3]                Euler angles output pitch,roll,yaw
 * @param[out] : float quaternion[4]         quaternion output w x y z
 * @param[out] : float line_accel[3]         none
 * @return: is_update = 1 else = 0
 */
unsigned char qst_fusion_update(float fusion_accel[3], float fusion_gyro[3], float *fusion_dt, float *rpy, float *quaternion, float *line_accel);

void init_state_recognition(int (*read)(unsigned char, unsigned char *, unsigned short));

void qst_set_inityaw(float init_yaw);

#endif
