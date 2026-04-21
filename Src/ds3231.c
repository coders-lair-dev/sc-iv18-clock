/*
 * ds3231.c
 *
 *  Created on: 18 июл. 2018 г.
 *      Author: scahr
 */
#include "ds3231.h"

volatile DS3231_DATE_TypeDef ds3231_Date;
volatile DS3231_TIME_TypeDef ds3231_Time;
volatile DS3231_TEMP_TypeDef ds3231_Temp;

volatile uint8_t ds3231_buffer[8];


uint8_t RTC_ConvertFromDec(uint8_t c) {
    uint8_t ch = ((c >> 4) * 10 + (0x0F & c));
    return ch;
}

uint8_t RTC_ConvertFromBinDec(uint8_t c) {
    uint8_t ch = ((c / 10) << 4) | (c % 10);
    return ch;
}

void I2C_WriteBuffer(I2C_HandleTypeDef *handle, uint8_t addr, uint8_t size) {
    HAL_I2C_Master_Transmit(handle, (uint16_t)addr, (uint8_t *)&ds3231_buffer, (uint16_t)size,
                            (uint32_t)DS3231_TRANSMIT_TIMEOUT);
}

void I2C_ReadBuffer(I2C_HandleTypeDef *handle, uint8_t addr, uint8_t size) {
    HAL_I2C_Master_Receive(handle, (uint16_t)addr, (uint8_t *)&ds3231_buffer, (uint16_t)size,
                           (uint32_t)DS3231_TRANSMIT_TIMEOUT);
}

uint8_t I2C_Write_Byte(I2C_HandleTypeDef *handle, uint8_t dev_addr, uint8_t addr, uint8_t data) {
    uint8_t buf[] = {addr, data};

    uint8_t d;

    while (HAL_I2C_GetState(handle) != HAL_I2C_STATE_READY)
        ;

    d = HAL_I2C_Master_Transmit(handle, (uint16_t)dev_addr, buf, 2, (uint32_t)DS3231_TRANSMIT_TIMEOUT);
    if (d != HAL_OK) {
        return d;
    }

    return HAL_OK;
}

uint8_t I2C_Read_Byte(I2C_HandleTypeDef *handle, uint8_t DEV_ADDR, uint8_t addr) {
    uint8_t data = 0;
    uint8_t d;
    while (HAL_I2C_GetState(handle) != HAL_I2C_STATE_READY)
        ;
    d = HAL_I2C_Master_Transmit(handle, (uint16_t)DEV_ADDR, &addr, 1, (uint32_t)DS3231_TRANSMIT_TIMEOUT);
    if (d != HAL_OK) {
        return d;
    }

    while (HAL_I2C_GetState(handle) != HAL_I2C_STATE_READY)
        ;
    d = HAL_I2C_Master_Receive(handle, (uint16_t)DEV_ADDR, &data, 1, (uint32_t)DS3231_TRANSMIT_TIMEOUT);
    if (d != HAL_OK) {
        return d;
    }
    return data;
}

void DS3231_ReadData() {
    ds3231_buffer[0] = 0;
    I2C_WriteBuffer(&hi2c1, (uint16_t)DS3231_ADDR << 1, 1);    //Передадим адрес устройству

    I2C_ReadBuffer(&hi2c1, (uint16_t)DS3231_ADDR << 1, 7);

    ds3231_Date.year = RTC_ConvertFromDec(ds3231_buffer[6]);
    ds3231_Date.month = RTC_ConvertFromDec(ds3231_buffer[5]);
    ds3231_Date.date = RTC_ConvertFromDec(ds3231_buffer[4]);
    ds3231_Date.day = RTC_ConvertFromDec(ds3231_buffer[3]);

    ds3231_Time.hour = RTC_ConvertFromDec(ds3231_buffer[2]);
    ds3231_Time.minutes = RTC_ConvertFromDec(ds3231_buffer[1]);
    ds3231_Time.seconds = RTC_ConvertFromDec(ds3231_buffer[0]);

    ds3231_Temp.temp_1 = I2C_Read_Byte(&hi2c1, (uint16_t)DS3231_ADDR << 1, DS3231_T_MSB);
    uint8_t temp2 = I2C_Read_Byte(&hi2c1, (uint16_t)DS3231_ADDR << 1, DS3231_T_LSB);

    temp2 = (temp2 / 128);    // сдвигаем на 6 - точность 0,25 (2 бита)
    // сдвигаем на 7 - точность 0,5 (1 бит)
    ds3231_Temp.temp_2 = temp2 * 5;
}

void DS3231_SetDateTime(uint8_t year,
                        uint8_t month,
                        uint8_t date,
                        uint8_t day_of_week,
                        uint8_t hour,
                        uint8_t minutes,
                        uint8_t seconds) {
    I2C_Write_Byte(&hi2c1, (uint16_t)DS3231_ADDR << 1, DS3231_DAY, RTC_ConvertFromBinDec(day_of_week));
    I2C_Write_Byte(&hi2c1, (uint16_t)DS3231_ADDR << 1, DS3231_DATE, RTC_ConvertFromBinDec(date));
    I2C_Write_Byte(&hi2c1, (uint16_t)DS3231_ADDR << 1, DS3231_MONTH, RTC_ConvertFromBinDec(month));
    I2C_Write_Byte(&hi2c1, (uint16_t)DS3231_ADDR << 1, DS3231_YEAR, RTC_ConvertFromBinDec(year));

    I2C_Write_Byte(&hi2c1, (uint16_t)DS3231_ADDR << 1, DS3231_HOURS, RTC_ConvertFromBinDec(hour));
    I2C_Write_Byte(&hi2c1, (uint16_t)DS3231_ADDR << 1, DS3231_MINUTES, RTC_ConvertFromBinDec(minutes));
    I2C_Write_Byte(&hi2c1, (uint16_t)DS3231_ADDR << 1, DS3231_SECONDS, RTC_ConvertFromBinDec(seconds));
}
