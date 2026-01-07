#ifndef _APP_FLASH_H_
#define _APP_FLASH_H_

#define INT16_DATA_LEN 6
#define CALI_DATA_LEN 8
typedef struct
{
    int16_t p[INT16_DATA_LEN];
} ParamData_t;

typedef struct app_flash
{
    /* data */
    uint8_t c[CALI_DATA_LEN];  // 实际只有7个字节有效
}CaliData_t;

#define PARAM_DATA_SIZE sizeof(ParamData_t) // 8 bytes
// 0x20000
#define PARAM_PAGE_ADDR 0x0801F800         
// 最后一页
#define CALI_PAGE_ADDR 0x0801F000           
// 倒数第二页 每页是 2KB

// uint8_t Flash_EraseParamPage(void);
uint8_t Flash_EraseParamPage(uint32_t page_start_add);

uint8_t Flash_WriteParams(ParamData_t *param);

uint8_t save_params(ParamData_t *param);

void save_6_int16(void);

void check_if_data_been_write(void);

uint8_t save_8_cali_data(uint8_t *cali_data);

void read_8_cali_data(uint8_t *cali_data);

extern uint8_t data_been_write;

#endif
