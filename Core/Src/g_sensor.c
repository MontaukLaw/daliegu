#include "user_comm.h"

// icm42688RawData_t acc, gyro;
// icm42688Float3_t acc_g, gyro_dps;
// icm42688RawData_t acc_mapped, gyro_mapped;

float q_out[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // 初始四元数
float line_acc[3] = {0, 0, 0};
qmi8658_state g_imu = {0};

static float gyro[3];
static float accel[3];

static float gyro_mapped[3];
static float accel_mapped[3];

uint16_t static_cali_count = 0;
uint8_t static_cali_flag = 0;
static float euler_angle[3] = {0, 0, 0};

int16_t static_offset_gyro[3] = {0, 0, 0};
int16_t static_offset_acc[3] = {0, 0, 0};

int32_t accel_static_calibration_sum[3] = {0, 0, 0};
int32_t gyro_static_calibration_sum[3] = {0, 0, 0};

float gyro_raw_static_deviation = 0;
float accel_raw_static_deviation = 0;

// 10ms执行一次.
void gsensor_task(void)
{
    icm42688RawData_t acc_mapped = {0};
    icm42688RawData_t gyro_mapped = {0};
    icm42688RawData_t acc = {0}, gyro = {0};

    static uint32_t last_run_tck = 0;
    uint32_t now = HAL_GetTick();

    // 调节帧率
    if (now - last_run_tck < 10)
        return;

    icm42688_read_accel(&acc);
    icm42688_read_gyro(&gyro);

    // 转向映射
    acc_mapped = map_vec3_by_orientation(acc);
    gyro_mapped = map_vec3_by_orientation(gyro);

    float dt = (now - last_run_tck) * 0.001f; // s

    icm42688_pipeline_update(&acc_mapped, &gyro_mapped, dt);

    // char tx_buf[96];
    // int n = snprintf(tx_buf, sizeof(tx_buf),
    //                  "%.4f, %.4f, %.4f, %.4f\r\n",
    //                  g_q[0], g_q[1], g_q[2], g_q[3]);
    // if (n > 0)
    // {
    //     HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, (uint16_t)n, 50);
    // }

    last_run_tck = now;
}
/**
 * @brief   在QMI8658指定寄存器地址读出一个数据
 * @param   reg:        寄存器地址
 * @retval  读到的数据
 */
uint8_t atk_qmi8658_read_byte(uint8_t reg)
{
    uint8_t data = 0;
    HAL_I2C_Master_Transmit(&hi2c3, QMI8658_ADDR << 1, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c3, QMI8658_ADDR << 1, &data, 1, HAL_MAX_DELAY);
    return data;
}

/**
 * @brief   从QMI8568读取N字节数据
 * @param   reg:        寄存器地址
 * @param   date:       数据存储buf
 * @param   len:        数据长度
 * @retval  读出结果
 * @retval  0, 操作成功
 *          其他, 操作失败
 */
int atk_qmi8568_read_nbytes(uint8_t reg, uint8_t *date, unsigned short len)
{
    HAL_I2C_Master_Transmit(&hi2c3, QMI8658_ADDR << 1, &reg, 1, HAL_MAX_DELAY);
    return HAL_I2C_Master_Receive(&hi2c3, QMI8658_ADDR << 1, date, len, HAL_MAX_DELAY);
}

/**
 * @brief   写一个字节到QMI8568的寄存器
 * @param   reg: 寄存器地址
 * @param   data: 寄存器数据
 * @retval  写入结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t atk_qmi8658_write_byte(uint8_t reg, uint8_t data)
{
    uint8_t tx[2];
    tx[0] = reg;
    tx[1] = data;
    return HAL_I2C_Master_Transmit(&hi2c3, QMI8658_ADDR << 1, tx, 2, HAL_MAX_DELAY);
}

void print_offset(void)
{
    printf("%d,%d,%d,%d,%d,%d\r\n", static_offset_acc[0], static_offset_acc[1], static_offset_acc[2], static_offset_gyro[0], static_offset_gyro[1], static_offset_gyro[2]);
}

/**
 * @brief   使能陀螺仪、加速度计
 * @param   enableFlags ：
 *          QMI8658_DISABLE_ALL  : 都不使能
 *          QMI8658_ACC_ENABLE   : 使能加速度计
 *          QMI8658_GYR_ENABLE   : 使能陀螺仪
 *          QMI8658_ACCGYR_ENABLE: 使能陀螺仪、加速度计
 * @retval  无
 */
void atk_qmi8658_enablesensors(unsigned char enableFlags)
{
#if defined(QMI8658_SYNC_SAMPLE_MODE)
    g_imu.cfg.syncsample = 1;
    atk_qmi8658_enable_ahb_clock(0);
    atk_qmi8658_write_byte(Register_Ctrl7, enableFlags | 0x80);
#else
    atk_qmi8658_write_byte(Register_Ctrl7, enableFlags);
#endif
    g_imu.cfg.ensensors = enableFlags & 0x03;
    delay_ms(2);
}

uint8_t imu_static_calibration(short acc_raw[3], short gyro_raw[3])
{
    uint8_t axis;
    if (static_cali_flag == 1)
        return 1;

    // 检查是否flash已经有了数据
    if (data_been_write == 1)
    {
        static_cali_flag = 1;
        return 1;
    }

    if (static_cali_count == 0)
    {
        memset((void *)accel_static_calibration_sum, 0, sizeof(accel_static_calibration_sum));
        memset((void *)gyro_static_calibration_sum, 0, sizeof(gyro_static_calibration_sum));
        static_cali_count++;
    }
    else if (static_cali_count < MAX_STATIC_CALI_COUNTER)
    {
        for (axis = 0; axis < 3; axis++)
        {
            gyro_static_calibration_sum[axis] += gyro_raw[axis];
            if (axis == 2)
            {
                accel_static_calibration_sum[axis] += (acc_raw[axis] - g_imu.ssvt_a);
            }
            else
            {
                accel_static_calibration_sum[axis] += acc_raw[axis];
            }
        }
        static_cali_count++;
    }
    else if (static_cali_count == MAX_STATIC_CALI_COUNTER)
    {
        for (axis = 0; axis < 3; axis++)
        {
            static_offset_acc[axis] = (accel_static_calibration_sum[axis] / (MAX_STATIC_CALI_COUNTER - 1));
            static_offset_gyro[axis] = (gyro_static_calibration_sum[axis] / (MAX_STATIC_CALI_COUNTER - 1));
        }

        // 保存数据
        save_6_int16();
        static_cali_count = 0;
        return 1;
    }
    return 0;
}

/**
 * @brief   传感器软件复位
 * @param   无
 * @retval  无
 */
void atk_qmi8658_reset(void)
{
    atk_qmi8658_write_byte(Register_Reset, 0xB0); /* 复位QMI8658 */
    delay_ms(150);
}

/**
 * @brief   检查QMI8658A的ID
 * @param   无
 * @retval  检查结果
 * @arg     0: 正确
 * @arg     1: 错误
 */
uint8_t atk_qmi8658_check_whoami(void)
{
    uint8_t qmi8658_chip_id = 0;
    uint8_t qmi8658_revision_id = 0;
    uint8_t i = 0;

    while ((qmi8658_chip_id != 0x05) && (i < 5)) /* 多次尝试检查设备标识符 */
    {
        qmi8658_chip_id = atk_qmi8658_read_byte(Register_WhoAmI);
        if (qmi8658_chip_id == 0x05)
        {
            qmi8658_revision_id = atk_qmi8658_read_byte(Register_Revision); /* 读取设备ID */
            break;
        }
        i++;
    }
    if ((qmi8658_chip_id == 0x05) && (qmi8658_revision_id == 0x7c)) /* 读取到准确的标识符和设备ID */
    {
        printf("qmi8658 chip id: %#x, device id: %#x\r\n", qmi8658_chip_id, qmi8658_revision_id);
        return 0;
    }
    else
    {
        printf("qmi8658 chip id error: %#x, device id: %#x\r\n", qmi8658_chip_id, qmi8658_revision_id);
        return 1;
    }
}

#if 0
/**
 * @brief   陀螺仪校准
 * @param   无
 * @retval  检查结果
 * @arg     0: 校准成功
 * @arg     1: 校准失败
 */
uint8_t atk_qmi8658_calibration(void)
{
    uint8_t sta = 0;
    atk_qmi8658_write_byte(Register_Ctrl7, 0x00); /* 关闭陀螺仪、加速度计 */
    atk_qmi8658_write_byte(Register_Ctrl9, 0xA2);

    delay_ms(2000);
    sta = atk_qmi8658_read_byte(Register_COD_Status);
    if (sta == 0x00)
    {
        return 0;
    }
    else
        return 1;
}
#endif

void qmi8658_get_gyro_gain(unsigned char *cod_data)
{
    atk_qmi8568_read_nbytes(Register_dVX_L, cod_data, 6);
    printf("cod data[0x%x 0x%x 0x%x 0x%x 0x%x 0x%x]\r\n", cod_data[0], cod_data[1], cod_data[2],
           cod_data[3], cod_data[4], cod_data[5]);
}

int qmi8658_send_ctl9cmd(enum Ctrl9Command cmd)
{
    unsigned char status1 = 0x00;
    unsigned short count = 0;
    unsigned char status_reg = Register_StatusInt;
    unsigned char cmd_done = 0x80;
    unsigned char retry = 0;
    int ret1 = 0;
    int ret2 = 0;

#if defined(QMI8658_SYNC_SAMPLE_MODE)
    if (g_imu.cfg.syncSample == 1)
    {
        status_reg = Register_Status1;
        cmd_done = 0x01;
    }
#endif
    while (retry++ < 3)
    {
        atk_qmi8658_write_byte(Register_Ctrl9, (unsigned char)cmd); // write commond to ctrl9

        atk_qmi8568_read_nbytes(status_reg, &status1, 1);
        while (((status1 & cmd_done) != cmd_done) && (count++ < 100)) // read statusINT until bit7 is 1
        {
            delay_ms(1);
            atk_qmi8568_read_nbytes(status_reg, &status1, 1);
        }
        // printf("ctrl9 cmd (%d) done1 count=%d\n", cmd, count);
        if (count < 100)
        {
            ret1 = 1;
        }
        else
        {
            ret1 = 0;
        }

        atk_qmi8658_write_byte(Register_Ctrl9, Ctrl9_Cmd_Ack); // write commond  0x00 to ctrl9
        count = 0;
        atk_qmi8568_read_nbytes(status_reg, &status1, 1);
        while (((status1 & cmd_done) == cmd_done) && (count++ < 100)) // read statusINT until bit7 is 0
        {
            delay_ms(1); // 1 ms
            atk_qmi8568_read_nbytes(status_reg, &status1, 1);
        }
        // printf("ctrl9 cmd (%d) done2 count=%d\n", qmi8658_Ctrl9_Cmd_Ack, count);
        if (count < 100)
        {
            ret2 = 1;
        }
        else
        {
            ret2 = 0;
        }

        if ((ret1 == 0) || (ret2 == 0))
        {
            continue;
        }
        else
        {
            break;
        }
    }

    if (ret1 && ret2)
    {
        return 1;
    }
    else
    {
        printf("qmi8658_send_ctl9cmd fail cmd=%d\r\n", cmd);
        return 0;
    }
}

void qmi8658_apply_gyr_gain(unsigned char cod_data[6])
{
    atk_qmi8658_write_byte(Register_Ctrl7, 0x00);
    atk_qmi8658_write_byte(Qmi8658Register_Cal1_L, cod_data[0]);
    atk_qmi8658_write_byte(Qmi8658Register_Cal1_H, cod_data[1]);
    atk_qmi8658_write_byte(Qmi8658Register_Cal2_L, cod_data[2]);
    atk_qmi8658_write_byte(Qmi8658Register_Cal2_H, cod_data[3]);
    atk_qmi8658_write_byte(Qmi8658Register_Cal3_L, cod_data[4]);
    atk_qmi8658_write_byte(Qmi8658Register_Cal3_H, cod_data[5]);

    qmi8658_send_ctl9cmd(Ctrl9_Cmd_ApplyGyroGains);
}

void Flash_Read_COD(uint8_t *cod_data)
{
    // Flash_Read(0x0801FC00, cod_data, 7);;//把cod data从flash 中读取出来
    memset(cod_data, 0, 7);
    read_8_cali_data(cod_data);
    printf("Read COD from FLASH: \r\n");
    for (uint8_t i = 0; i < 7; i++)
    {
        printf("[%d]=0x%02x ", i, cod_data[i]);
    }
    printf("\r\n");
}

void Flash_Write_COD(uint8_t *cod_data)
{

    printf("Write COD to FLASH: \r\n");
    for (uint8_t i = 0; i < 7; i++)
    {
        printf("[%d]=0x%02x ", i, cod_data[i]);
    }
    printf("\r\n");

    save_8_cali_data(cod_data);

    // Flash_Write(0x0801FC00,cod_data, 7); 存cod data到FLASH
}

/**
 * @brief   陀螺仪校准
 * @param   无
 * @retval  检查结果
 * @arg     0: 校准成功
 * @arg     1: 校准失败
 */
uint8_t atk_qmi8658_calibration(void)
{
    uint8_t sta = 0;
    uint8_t cod[CALI_DATA_LEN] = {0};
    atk_qmi8658_write_byte(Register_Ctrl9, (unsigned char)Ctrl9_Cmd_OnDemandCalivration);
    delay_ms(2000);
    atk_qmi8658_write_byte(Register_Ctrl9, (unsigned char)Ctrl9_Cmd_Ack);
    delay_ms(10);
    sta = atk_qmi8658_read_byte(Register_COD_Status);
    if (sta == 0x00) // 校准成功
    {
        qmi8658_get_gyro_gain(cod); // 获取校准值
        printf("qmi8658_on_demand_cali done! cod[%02x %02x %02x %02x %02x %02x]\r\n", cod[0], cod[1], cod[2], cod[3], cod[4], cod[5]);
        cod[6] = 1;           // 有效值标志位
        Flash_Write_COD(cod); // 校准成功重新将值保存到FLASH内

        // 反读取一下看看是否写入成功
        Flash_Read_COD(cod);

        return 0;
    }
    else // 校准失败
    {
        Flash_Read_COD(cod);
        printf("qmi8658_apply_cod_par![%d %d %d]\r\n",
               (unsigned short)(cod[1] << 8 | cod[0]),
               (unsigned short)(cod[3] << 8 | cod[2]),
               (unsigned short)(cod[5] << 8 | cod[4]));

        if (cod[6] == 1) // 确认FLASH 保存的是有效值
        {
            qmi8658_apply_gyr_gain(cod); // 写校准值到芯片内部，掉电会丢失
        }
    }
    return 0;
}

/**
 * @brief   配置陀螺仪参数
 * @param   range       ：量程
 * @param   odr         ：odr输出速率
 * @param   lpfEnable   ：低通滤波器 ：Qmi8658Lpf_Enable 打开，Qmi8658Lpf_Disable 关闭
 * @param   stEnable    ：陀螺仪自检 ：Qmi8658St_Enable 自检，Qmi8658St_Disable 不自检
 * @retval  无
 */
void atk_qmi8658_config_gyro(enum qmi8658_gyrrange range, enum qmi8658_gyrodr odr, enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
    /* Set the CTRL3 register to configure dynamic range and ODR */
    unsigned char ctl_dada;

    /* Store the scale factor for use when processing raw data */
    switch (range)
    {
    case Qmi8658gyrrange_16dps:
        g_imu.ssvt_g = 2048;
        break;
    case Qmi8658gyrrange_32dps:
        g_imu.ssvt_g = 1024;
        break;
    case Qmi8658gyrrange_64dps:
        g_imu.ssvt_g = 512;
        break;
    case Qmi8658gyrrange_128dps:
        g_imu.ssvt_g = 256;
        break;
    case Qmi8658gyrrange_256dps:
        g_imu.ssvt_g = 128;
        break;
    case Qmi8658gyrrange_512dps:
        g_imu.ssvt_g = 64;
        break;
    case Qmi8658gyrrange_1024dps:
        g_imu.ssvt_g = 32;
        break;
    case Qmi8658gyrrange_2048dps:
        g_imu.ssvt_g = 16;
        break;
    default:
        range = Qmi8658gyrrange_512dps;
        g_imu.ssvt_g = 64;
        break;
    }

    if (stEnable == Qmi8658St_Enable)
    {
        ctl_dada = (unsigned char)range | (unsigned char)odr | 0x80;
    }
    else
    {
        ctl_dada = (unsigned char)range | (unsigned char)odr;
    }
    atk_qmi8658_write_byte(Register_Ctrl3, ctl_dada);

    /* Conversion from degrees/s to rad/s if necessary */
    /* set LPF & HPF */
    atk_qmi8568_read_nbytes(Register_Ctrl5, &ctl_dada, 1);
    ctl_dada &= 0x0f;
    if (lpfEnable == Qmi8658Lpf_Enable)
    {
        ctl_dada |= G_LSP_MODE_3;
        ctl_dada |= 0x10;
    }
    else
    {
        ctl_dada &= ~0x10;
    }
    atk_qmi8658_write_byte(Register_Ctrl5, ctl_dada);
}

/*****************************************************************************************************************/
/**
 * @brief   配置加速度计参数
 * @param   range       ：量程
 * @param   odr         ：odr输出速率
 * @param   lpfEnable   ：低通滤波器 ：Qmi8658Lpf_Enable 打开，Qmi8658Lpf_Disable 关闭
 * @param   stEnable    ：陀螺仪自检 ：Qmi8658St_Enable 自检，Qmi8658St_Disable 不自检
 * @retval  无
 */
void atk_qmi8658_config_acc(enum qmi8658_accrange range, enum qmi8658_accodr odr, enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
    unsigned char ctl_dada;

    switch (range)
    {
    case Qmi8658accrange_2g:
        g_imu.ssvt_a = (1 << 14);
        break;
    case Qmi8658accrange_4g:
        g_imu.ssvt_a = (1 << 13);
        break;
    case Qmi8658accrange_8g:
        g_imu.ssvt_a = (1 << 12);
        break;
    case Qmi8658accrange_16g:
        g_imu.ssvt_a = (1 << 11);
        break;
    default:
        range = Qmi8658accrange_8g;
        g_imu.ssvt_a = (1 << 12);
        break;
    }
    if (stEnable == Qmi8658St_Enable)
    {
        ctl_dada = (unsigned char)range | (unsigned char)odr | 0x80;
    }
    else
    {
        ctl_dada = (unsigned char)range | (unsigned char)odr;
    }
    atk_qmi8658_write_byte(Register_Ctrl2, ctl_dada);
    /* set LPF & HPF */
    atk_qmi8568_read_nbytes(Register_Ctrl5, &ctl_dada, 1);
    ctl_dada &= 0xf0;
    if (lpfEnable == Qmi8658Lpf_Enable)
    {
        ctl_dada |= A_LSP_MODE_3;
        ctl_dada |= 0x01;
    }
    else
    {
        ctl_dada &= ~0x01;
    }
    atk_qmi8658_write_byte(Register_Ctrl5, ctl_dada);
}

/**
 * @brief   配置QMI8658陀螺仪和加速度计的量程、输出频率参数等
 * @param   low_power ： 0： 正常模式 1：低功耗模式
 * @retval  无
 */
void atk_qmi8658_config_reg(unsigned char low_power)
{
    atk_qmi8658_enablesensors(QMI8658_DISABLE_ALL);
    if (low_power)
    {
        g_imu.cfg.ensensors = QMI8658_ACC_ENABLE;
        g_imu.cfg.accrange = Qmi8658accrange_8g;
        g_imu.cfg.accodr = Qmi8658accodr_LowPower_21Hz;
        g_imu.cfg.gyrrange = Qmi8658gyrrange_1024dps;
        g_imu.cfg.gyrodr = Qmi8658gyrodr_250Hz;
    }
    else
    {
        g_imu.cfg.ensensors = QMI8658_ACCGYR_ENABLE;  /* 使能陀螺仪、加速度计 */
        g_imu.cfg.accrange = Qmi8658accrange_16g;     /* ±16g */
        g_imu.cfg.accodr = Qmi8658accodr_500Hz;       /* 500Hz采样 */
        g_imu.cfg.gyrrange = Qmi8658gyrrange_2048dps; // Qmi8658gyrrange_128dps; /* ±128dps */
        g_imu.cfg.gyrodr = Qmi8658gyrodr_500Hz;       /* 500Hz采样 */
    }

    if (g_imu.cfg.ensensors & QMI8658_ACC_ENABLE)
    {
        atk_qmi8658_config_acc(g_imu.cfg.accrange, g_imu.cfg.accodr, Qmi8658Lpf_Disable, Qmi8658St_Enable); /* 设置参数并开启加速度计自检和低通滤波器 */
    }
    if (g_imu.cfg.ensensors & QMI8658_GYR_ENABLE)
    {
        atk_qmi8658_config_gyro(g_imu.cfg.gyrrange, g_imu.cfg.gyrodr, Qmi8658Lpf_Disable, Qmi8658St_Enable); /* 设置参数并开启陀螺仪自检和低通滤波器 */
    }
}

void reset_q_out(void)
{
    q_out[0] = 1.0f;
    q_out[1] = 0.0f;
    q_out[2] = 0.0f;
    q_out[3] = 0.0f;
}

/**
 * @brief   初始化QMI8658
 * @param   无
 * @retval  初始化结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t atk_qmi8658_init(void)
{
    // delay_ms(2000);
    atk_qmi8658_reset(); /* 复位传感器 */

    if (atk_qmi8658_check_whoami()) /* 检查设备ID是否正确 */
    {
        printf("qmi8658 whoami error\r\n");
        return 1;
    }

    atk_qmi8658_write_byte(Register_Ctrl1, 0x60); // 开启寄存器自增模式

    if (atk_qmi8658_calibration())
    {
        printf("qmi8658 calibration error\r\n");
        return 1;
    }

    printf("calibration done\r\n");

    atk_qmi8658_write_byte(Register_Ctrl7, 0x00); /* 关闭陀螺仪、加速度计 */

    atk_qmi8658_config_reg(0);                      /* 配置陀螺仪和加速度计的量程和数据输出速率等参数 */
    atk_qmi8658_enablesensors(g_imu.cfg.ensensors); /* 使能陀螺仪、加速度计 */

    gyro_raw_static_deviation = 0.005f * 57.29578f * g_imu.ssvt_g;
    accel_raw_static_deviation = 0.04f / 9.8f * g_imu.ssvt_a;

    init_state_recognition(&atk_qmi8568_read_nbytes);

    qst_vqf_init(0.005f);

    printf("QMI8658A Ready!\r\n");

    return 0;
}

/**
 * @brief   读取补偿后QMI8658陀螺仪和加速度的数据（无判断）
 * @param   acc  ：存储加速度计xyz轴数据
 * @param   gyro ：存储陀螺仪xyz轴数据
 * @retval  无
 */
void atk_qmi8658_read_sensor_data(float *acc, float *gyro)
{
    unsigned char buf_reg[12];
    short raw_acc_xyz[3];
    short raw_gyro_xyz[3];
    unsigned char axis = 0;
    static int cali_count = 0;
    static float offset_acc[3] = {0.0, 0.0, 0.0};
    static float offset_gyro[3] = {0.0, 0.0, 0.0};
    static float accel_calibration_sum[3] = {0.0f, 0.0f, 0.0f};
    static float gyro_calibration_sum[3] = {0.0f, 0.0f, 0.0f};

    float acc_raw[3];
    float gyro_raw[3];

    /* 读取加速度计和陀螺仪数据 */
    atk_qmi8568_read_nbytes(Register_Ax_L, buf_reg, 12);

    raw_acc_xyz[0] = (short)((unsigned short)(buf_reg[1] << 8) | (buf_reg[0]));
    raw_acc_xyz[1] = (short)((unsigned short)(buf_reg[3] << 8) | (buf_reg[2]));
    raw_acc_xyz[2] = (short)((unsigned short)(buf_reg[5] << 8) | (buf_reg[4]));

    raw_gyro_xyz[0] = (short)((unsigned short)(buf_reg[7] << 8) | (buf_reg[6]));
    raw_gyro_xyz[1] = (short)((unsigned short)(buf_reg[9] << 8) | (buf_reg[8]));
    raw_gyro_xyz[2] = (short)((unsigned short)(buf_reg[11] << 8) | (buf_reg[10]));

    // IMU校准校准校准时要求芯片丝印面向天，水平静止摆放
    if (imu_static_calibration(raw_acc_xyz, raw_gyro_xyz) == 1)
    {
        raw_acc_xyz[0] = raw_acc_xyz[0] - static_offset_acc[0];
        raw_acc_xyz[1] = raw_acc_xyz[1] - static_offset_acc[1];
        raw_acc_xyz[2] = raw_acc_xyz[2] - static_offset_acc[2];
        raw_gyro_xyz[0] = raw_gyro_xyz[0] - static_offset_gyro[0];
        raw_gyro_xyz[1] = raw_gyro_xyz[1] - static_offset_gyro[1];
        raw_gyro_xyz[2] = raw_gyro_xyz[2] - static_offset_gyro[2];
    }

    /* 加速度单位：m/s2 */
    acc[0] = (float)(raw_acc_xyz[0] * ONE_G) / g_imu.ssvt_a;
    acc[1] = (float)(raw_acc_xyz[1] * ONE_G) / g_imu.ssvt_a;
    acc[2] = (float)(raw_acc_xyz[2] * ONE_G) / g_imu.ssvt_a;

    /* 陀螺仪单位：rad/s */
    gyro[0] = (float)(raw_gyro_xyz[0] * M_PI) / (g_imu.ssvt_g * 180); /* *pi/180 */
    gyro[1] = (float)(raw_gyro_xyz[1] * M_PI) / (g_imu.ssvt_g * 180);
    gyro[2] = (float)(raw_gyro_xyz[2] * M_PI) / (g_imu.ssvt_g * 180);
}

void change_xy(float *src, float *dst)
{
    dst[0] = -src[1];
    dst[1] = src[0];
    dst[2] = src[2];
}

/**
 * @brief   判断数据更新后，在读取补偿后QMI8658陀螺仪和加速度的数据(推荐使用)
 * @param   acc  : 加速度计 X,Y,Z缓存区;
 * @param   gyro : 陀螺仪 X,Y,Z缓存区;
 * @retval  无
 */
void atk_qmi8658_read_xyz(float *acc, float *gyro)
{
    unsigned char status = 0;
    unsigned char data_ready = 0;
    int retry = 0;

    while (retry++ < 3)
    {

        // #if defined(QMI8658_SYNC_SAMPLE_MODE)
        //         atk_qmi8568_read_nbytes(Register_StatusInt, &status, 1);

        //         if (status & 0x01)
        //         {
        //             delay_us(12); /* delay 12us <=500Hz， 12us 1000Hz, 4us 2000Hz 2us > 2000Hz */
        //         }
        //         if ((status & 0x01) || (status & 0x03))
        //         {
        //             data_ready = 1;
        //             break;
        //         }
        // #else
        /* 检查加速度计和陀螺仪数据是否可用 */
        atk_qmi8568_read_nbytes(Register_Status0, &status, 1);
        if (status & 0x03)
        {
            data_ready = 1;
            break;
        }
        HAL_Delay(2);

        // #endif
    }
    if (data_ready)
    {
        atk_qmi8658_read_sensor_data(acc, gyro);

        g_imu.imu[0] = acc[0]; // x
        g_imu.imu[1] = acc[1]; // y
        g_imu.imu[2] = acc[2]; // z
        g_imu.imu[3] = gyro[0];
        g_imu.imu[4] = gyro[1];
        g_imu.imu[5] = gyro[2];

        // g_imu.imu[0] = -acc[1];  // -y
        // g_imu.imu[1] = acc[0];   // x
        // g_imu.imu[2] = acc[2];   // z
        // g_imu.imu[3] = -gyro[1]; // x
        // g_imu.imu[4] = gyro[0];  // y
        // g_imu.imu[5] = gyro[2];
        static uint32_t cnter = 0;
        cnter++;
        if (cnter > 100)
        {
            cnter = 0;
            // printf("acc: %.4f, %.4f, %.4f, gyro: %.4f, %.4f, %.4f\r\n", acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
        }
        // printf("acc: %.4f, %.4f, %.4f, gyro: %.4f, %.4f, %.4f\r\n", acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
    }
    else
    {

        // printf("data not ready status: 0x%2x\r\n", status);

        acc[0] = g_imu.imu[0];
        acc[1] = g_imu.imu[1];
        acc[2] = g_imu.imu[2];
        gyro[0] = g_imu.imu[3];
        gyro[1] = g_imu.imu[4];
        gyro[2] = g_imu.imu[5];
        /* 调试使用 */
    }
}

void qmi8658_task(void)
{
    static uint32_t last = 0;
    uint32_t now = HAL_GetTick();

    // 改成 5ms（200Hz）
    if (now - last < 10)
        return;

    float dt = (now - last) * 0.001f;
    last = now;

    // 0.005
    // printf("dt: %.4f\r\n", dt);

    // 保护 dt
    // if (dt <= 0 || dt > 0.015f)
    // {
    //     // printf("qmi8658_task: dt error %.4f\r\n", dt);
    //     return;
    // }

    atk_qmi8658_read_xyz(accel, gyro);

    // 转xy轴
    change_xy(accel, accel_mapped);
    change_xy(gyro, gyro_mapped);

    if (static_cali_flag == 1)
    {
        qst_fusion_update(accel_mapped, gyro_mapped, &dt, euler_angle, q_out, line_acc);
    }
}