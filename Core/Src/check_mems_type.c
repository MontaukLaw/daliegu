#include "user_comm.h"

uint8_t mems_type = 0;

void check_mems_type(void)
{
    uint8_t who = icm42688_iic_read_reg(ICM42688_WHO_AM_I);
    if (who == ICM42688_ID)
    {
        mems_type = MEMS_TYPE_ICM42688;
        return;
    }

    if (atk_qmi8658_check_whoami() == 0)
    {
        mems_type = MEMS_TYPE_QMI8658;
    }
    else
    {
        mems_type = MEMS_TYPE_UNKNOWN;
    }
}

void init_mems(void)
{

    if (mems_type == MEMS_TYPE_ICM42688)
    {
        icm42688_init();
    }
    else if (mems_type == MEMS_TYPE_QMI8658)
    {
        atk_qmi8658_init();
    }
}

void mems_task(void)
{
    if (mems_type == MEMS_TYPE_ICM42688)
    {
        gsensor_task();
    }
    else if (mems_type == MEMS_TYPE_QMI8658)
    {
        qmi8658_task();
    }
}
