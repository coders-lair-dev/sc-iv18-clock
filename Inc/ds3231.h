/*
 * ds3231.h
 *
 *  Created on: 18 июл. 2018 г.
 *      Author: scahr
 */

#ifndef DS3231_H_
#define DS3231_H_

#include "main.h"
#include "i2c.h"

typedef struct DS3231_DATE {
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint8_t year;
} DS3231_DATE_TypeDef;

typedef struct DS3231_TIME {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hour;
} DS3231_TIME_TypeDef;

typedef struct DS3231_TEMP {
    uint8_t temp_1;
    uint8_t temp_2;
} DS3231_TEMP_TypeDef;

#define DS3231_ADDR             0x68    // I2C 7-bit address of DS3231 RTC
#define DS3231_TRANSMIT_TIMEOUT 15

extern volatile DS3231_DATE_TypeDef ds3231_Date;
extern volatile DS3231_TIME_TypeDef ds3231_Time;
extern volatile DS3231_TEMP_TypeDef ds3231_Temp;

extern volatile uint8_t ds3231_buffer[8];

// Адреса регистров

#define DS3231_SECONDS     0x00    // Адрес регистра секунд
#define DS3231_MINUTES     0x01    // Адрес регистра минут
#define DS3231_HOURS       0x02    // Адрес регистра часов
#define DS3231_DAY         0x03    // Адрес регистра дня недели
#define DS3231_DATE        0x04    // Адрес регистра числа
#define DS3231_MONTH       0x05    // Адрес регистра месяца
#define DS3231_YEAR        0x06    // Адрес регистра года
#define DS3231_ALARM_1_SEC 0x07    // Адрес регистра будильника 1 секунд
#define DS3231_ALARM_1_MIN 0x08    // Адрес регистра будильника 1 минут
#define DS3231_ALARM_1_HOR 0x09    // Адрес регистра будильника 1 часов
#define DS3231_ALARM_1_DAY 0x0A    // Адрес регистра будильника 1 дня недели
#define DS3231_ALARM_1_DAT 0x0A    // Адрес регистра будильника 1 числа
#define DS3231_ALARM_2_MIN 0x0B    // Адрес регистра будильника 2 минут
#define DS3231_ALARM_2_HOR 0x0C    // Адрес регистра будильника 2 часов
#define DS3231_ALARM_2_DAY 0x0D    // Адрес регистра будильника 2 вдя недели
#define DS3231_ALARM_2_DAT 0x0D    // Адрес регистра будильника 2 числа
#define DS3231_CONTROL     0x0E    // Адрес регистра управления
#define DS3231_STATUS      0x0F    // Адрес регистра состояния
#define DS3231_SET_CLOCK   0x10    // Адрес регистра корректировки частоты генератора времяни (0x81 - 0x7F)
#define DS3231_T_MSB       0x11    // Адрес регистра последней измеренной температуры старшиий байт
#define DS3231_T_LSB       0x12    // Адрес регистра последней измеренной температуры младшиий байт

// 12/24-hour format helpers (not used in current firmware — always 24-hour).
// Kept for future reference. See DS3231 datasheet, register 0x02 (Hours):
//
//   bit 6: 0 = 24-hour mode, 1 = 12-hour mode
//   bit 5: in 12-hour mode — AM/PM flag (0 = AM, 1 = PM)
//
// The constants below were planned as bit masks / set-values for working
// with that register, but real implementation never happened.
//
// #define DS3231_HOURS_12             0x40
// #define DS3231_24                   0x3F
// #define DS3231_AM                   0x5F
// #define DS3231_PM                   0x60
// #define DS3231_GET_24               0x02
// #define DS3231_GET_PM               0x01
// #define DS3231_GET_AM               0x00

// Register bit positions
//
// A1M1..A1M4: Alarm 1 mask bits (bit 7 of the respective alarm registers:
//             seconds, minutes, hours, day/date — used together to configure
//             Alarm 1 trigger mode, see DS3231 datasheet, Table 2).
// A2M2..A2M4: Alarm 2 mask bits (bit 7 of minutes, hours, day/date registers;
//             Alarm 2 has no seconds register).
// DY_DT:      bit 6 of alarm day/date register — selects between day-of-week
//             (1) and day-of-month (0) matching.

#define DS3231_A1M1  7
#define DS3231_A1M2  7
#define DS3231_A1M3  7
#define DS3231_A1M4  7
#define DS3231_A2M2  7
#define DS3231_A2M3  7
#define DS3231_A2M4  7
#define DS3231_DY_DT 6

// Control register (0x0E) bit positions
#define DS3231_EOSC  7    // Enable Oscillator (inverted: 0 = enabled)
#define DS3231_BBSQW 6    // Battery-Backed Square-Wave Enable
#define DS3231_CONV  5    // Convert Temperature (force TCXO update)
#define DS3231_INTCN 2    // Interrupt Control (0 = SQW, 1 = alarm INT)
#define DS3231_A2IE  1    // Alarm 2 Interrupt Enable
#define DS3231_A1IE  0    // Alarm 1 Interrupt Enable

// Status register (0x0F) bit positions
#define DS3231_OSF     7    // Oscillator Stop Flag
#define DS3231_EN32KHZ 3    // Enable 32 kHz output
#define DS3231_BSY     2    // Busy (TCXO conversion in progress)
#define DS3231_A2F     1    // Alarm 2 Flag
#define DS3231_A1F     0    // Alarm 1 Flag

// Temperature register (0x11) MSB bit position
#define DS3231_SIGN 7    // Sign bit of temperature (two's complement)

// Выбор частоты выходного сигнала INT/SQW

#define DS3231_SQW_OUT_1HZ  0x00    // 1 Гц
#define DS3231_SQW_OUT_1KHZ 0x08    // 1.024 кГц
#define DS3231_SQW_OUT_4KHZ 0x10    // 4.096 кГц
#define DS3231_SQW_OUT_8KHZ 0x18    // 8.192 кГц

// Логический уровень при выключеном SQW/OUT

#define DS3231_OUT_HIGHT 0x80
#define DS3231_OUT_LOW   0x00

// Вспомогательные данные функций

#define DS3231_ERR 0
#define DS3231_OK  1
#define DS3231_ON  1
#define DS3231_OFF 0

// Будильники
#define DS3231_ALARM_1      0x00
#define DS3231_ALARM_2      0x01
#define DS3231_ALARM_OFF    0x00
#define DS3231_ALARM_1_ON   0x01
#define DS3231_ALARM_2_ON   0x02
#define DS3231_ALARM_ALL_ON 0x03

void DS3231_ReadData();
void DS3231_ReadData(void);

void DS3231_ReadData(void);

/**
 * Set DS3231 date and time.
 *
 * Parameters are decimal values (not BCD); conversion to BCD
 * is performed internally via RTC_ConvertFromBinDec().
 */
void DS3231_SetDateTime(uint8_t year,           // Two-digit year (0..99), e.g. 23 for 2023
                        uint8_t month,          // 1..12
                        uint8_t date,           // 1..31
                        uint8_t day_of_week,    // 1..7 (1 = Monday by convention)
                        uint8_t hour,           // 0..23 (24-hour format)
                        uint8_t minutes,        // 0..59
                        uint8_t seconds         // 0..59
);


#endif /* DS3231_H_ */
