#include "user_comm.h"

uint8_t data_been_write = 0;
extern int16_t static_offset_gyro[];
extern int16_t static_offset_acc[];

void Flash_ReadParams(ParamData_t *param)
{
    memcpy(param, (void *)PARAM_PAGE_ADDR, PARAM_DATA_SIZE);
}

void read_6_int16(void)
{
    ParamData_t param;
    Flash_ReadParams(&param);
    uint16_t i;
    for (i = 0; i < INT16_DATA_LEN / 2; i++)
    {
        static_offset_acc[i] = param.p[i];
        static_offset_gyro[i] = param.p[i + INT16_DATA_LEN / 2];
    }

    print_offset();
}

void read_8_cali_data(uint8_t *cali_data)
{
    CaliData_t cali;
    memcpy(&cali, (void *)CALI_PAGE_ADDR, CALI_DATA_LEN);
    memcpy(cali_data, cali.c, CALI_DATA_LEN);
}

uint8_t Flash_EraseParamPage(uint32_t page_start_add)
{
    HAL_FLASH_Unlock();
    
    // 清理所有错误标志
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    FLASH_EraseInitTypeDef Erase = {0};
    uint32_t page_error = 0;

    Erase.TypeErase = FLASH_TYPEERASE_PAGES;
    Erase.Page = (page_start_add - FLASH_BASE) / FLASH_PAGE_SIZE;
    Erase.Banks = FLASH_BANK_1;
    Erase.NbPages = 1;

    // DBG_PRINTF("Erase.Page: %d\r\n", Erase.Page);

    if (HAL_FLASHEx_Erase(&Erase, &page_error) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return 1; // fail
    }

    HAL_FLASH_Lock();
    return 0; // success
}

// （必须 8 字节对齐写入）
uint8_t Flash_WriteParams(ParamData_t *param)
{
    HAL_FLASH_Unlock();

    uint8_t buffer[16] = {0}; // 16 字节对齐，防止越界
    memcpy(buffer, param, PARAM_DATA_SIZE);

    uint32_t addr = PARAM_PAGE_ADDR;

    // 写入第 1 个 8 字节
    uint64_t dword = 0;
    memcpy(&dword, buffer, 8);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, dword) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return 1;
    }

    // 写入第 2 个 8 字节（实际上只有 4 字节有效，但仍需写 8 字节）
    memcpy(&dword, buffer + 8, 8);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + 8, dword) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return 2;
    }

    HAL_FLASH_Lock();
    return 0;
}

uint8_t flash_write_cali_data(CaliData_t *cali)
{
    HAL_FLASH_Unlock();

    uint8_t buffer[16] = {0}; // 16 字节对齐，防止越界
    memcpy(buffer, cali, CALI_DATA_LEN);

    uint32_t addr = CALI_PAGE_ADDR;

    // 写入第 1 个 8 字节
    uint64_t dword = 0;
    memcpy(&dword, buffer, 8);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, dword) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return 1;
    }

    HAL_FLASH_Lock();
    return 0;
}

uint8_t save_params(ParamData_t *param)
{
    if (Flash_EraseParamPage(PARAM_PAGE_ADDR) != 0)
        return 1;

    if (Flash_WriteParams(param) != 0)
        return 2;

    return 0; // success
}

uint8_t save_cali_data(CaliData_t *cali)
{
    if (Flash_EraseParamPage(CALI_PAGE_ADDR) != 0)
    {
        DBG_PRINTF("Erase CALI PAGE ERROR\r\n");
        return 1;
    }

    // 要写8个字节.
    if (flash_write_cali_data(cali) != 0)
    {
        DBG_PRINTF("Write CALI DATA ERROR\r\n");
        return 2;
    }

    return 0;
}

void check_if_data_been_write(void)
{
    uint8_t data[24] = {0};
    memcpy(data, (void *)PARAM_PAGE_ADDR, PARAM_DATA_SIZE);
    uint16_t i;

    ;
    for (i = 0; i < PARAM_DATA_SIZE; i++)
    {
        if (data[i] != 0xff)
        {
            data_been_write = 1;
            break;
        }
    }

    if (data_been_write)
    {
        printf("Loading data in flash\r\n");
        read_6_int16();
    }
}

void save_6_int16(void)
{

    uint16_t i;

    check_if_data_been_write();

    // 如果flash里面没有数据, 就保存数据
    if (data_been_write == 0)
    {
        printf("save static cali data to flash\r\n");
        // 复制数据
        ParamData_t param;
        for (i = 0; i < INT16_DATA_LEN / 2; i++)
        {
            param.p[i] = static_offset_acc[i];
            param.p[i + INT16_DATA_LEN / 2] = static_offset_gyro[i];
        }

        save_params(&param);
    }
    else
    {
        printf("static cali data has been saved in flash\r\n");
        read_6_int16();
    }
}

uint8_t save_8_cali_data(uint8_t *cali_data)
{
    CaliData_t cali;
    memcpy(cali.c, cali_data, CALI_DATA_LEN);
    return save_cali_data(&cali);
}
