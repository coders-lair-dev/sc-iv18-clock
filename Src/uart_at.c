/*
 * uart_at.c
 *
 *  AT-command interface over USART1 (LL).
 *  Hayes-style commands for setting/reading DS3231 date/time/temp.
 */

#include "uart_at.h"
#include "main.h"
#include "ds3231.h"
#include "xprintf.h"
#include <string.h>

#define RX_BUF_SIZE 64
static volatile uint8_t rxBuf[RX_BUF_SIZE];    // Receive ring buffer
static volatile uint8_t rxHead = 0;
static volatile uint8_t rxTail = 0;

#define CMD_BUF_SIZE 24
static char cmdBuf[CMD_BUF_SIZE];
static uint8_t cmdLen = 0;

static uint8_t echoEnabled = 1; /* ATE1 default on */

static void uart_putc(char c) {
    while (!LL_USART_IsActiveFlag_TXE(USART1)) {
    }

    LL_USART_TransmitData8(USART1, (uint8_t)c);
}

static void at_reply(const char *s) {
    while (*s) {
        uart_putc(*s++);
    }
}

/**
 * ISO: 1=Mon..7=Sun
 * y - full year (e.g. 2026)
 * m - month (1..12)
 * d - day (1..31)
 */
static uint8_t day_of_week_iso(uint16_t y, uint8_t m, uint8_t d) {
    static const uint8_t t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    if (m < 3) {
        y -= 1;
    }

    uint8_t s = (uint8_t)((y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7);    // 0=Sun..6=Sat
    return (s == 0) ? 7 : s;                                                      // 1=Mon..7=Sun
}


/**
 * parse 2 ASCII digits -> 0..99, or -1 on error
 */
static int8_t parse2(const char *p) {
    if (p[0] < '0' || p[0] > '9' || p[1] < '0' || p[1] > '9') {
        return -1;
    }

    return (int8_t)((p[0] - '0') * 10 + (p[1] - '0'));
}

static void at_execute(char *cmd) {
    /* AT - link check */
    if (strcmp(cmd, "AT") == 0) {
        at_reply("OK\r\n");
        return;
    }

    /* ATE0 / ATE1 - echo off/on */
    if (strcmp(cmd, "ATE0") == 0) {
        echoEnabled = 0;
        at_reply("OK\r\n");
        return;
    }
    if (strcmp(cmd, "ATE1") == 0) {
        echoEnabled = 1;
        at_reply("OK\r\n");
        return;
    }

    /* ATI - identification */
    if (strcmp(cmd, "ATI") == 0) {
        at_reply("IV-18 VFD Clock, coders-lair.com\r\n");
        at_reply("OK\r\n");

        return;
    }

    /* AT+TIME=HHMMSS */
    if (strncmp(cmd, "AT+TIME=", 8) == 0 && strlen(cmd) == 14) {
        int8_t hh = parse2(cmd + 8), mm = parse2(cmd + 10), ss = parse2(cmd + 12);
        if (hh >= 0 && hh <= 23 && mm >= 0 && mm <= 59 && ss >= 0 && ss <= 59) {
            /* keep current date, change time only */
            DS3231_SetDateTime(ds3231_Date.year, ds3231_Date.month, ds3231_Date.date, ds3231_Date.day, (uint8_t)hh,
                               (uint8_t)mm, (uint8_t)ss);
            at_reply("OK\r\n");
        } else {
            at_reply("ERROR\r\n");
        }

        return;
    }

    /* AT+DATE=DDMMYY */
    if (strncmp(cmd, "AT+DATE=", 8) == 0 && strlen(cmd) == 14) {
        int8_t dd = parse2(cmd + 8), mo = parse2(cmd + 10), yy = parse2(cmd + 12);
        if (dd >= 1 && dd <= 31 && mo >= 1 && mo <= 12 && yy >= 0 && yy <= 99) {
            uint8_t dow = day_of_week_iso((uint16_t)(2000 + yy), (uint8_t)mo, (uint8_t)dd);
            DS3231_SetDateTime((uint8_t)yy, (uint8_t)mo, (uint8_t)dd, dow, ds3231_Time.hour, ds3231_Time.minutes,
                               ds3231_Time.seconds);
            at_reply("OK\r\n");
        } else {
            at_reply("ERROR\r\n");
        }

        return;
    }

    /* AT+DOW? - query day of week number (1=Mon..7=Sun) */
    if (strcmp(cmd, "AT+DOW?") == 0) {
        char b[8];

        xsprintf(b, "%d\n", ds3231_Date.day);
        at_reply(b);
        at_reply("OK\r\n");

        return;
    }

    /* AT+DOW=N - day of week 1..7*/
    if (strncmp(cmd, "AT+DOW=", 7) == 0 && strlen(cmd) == 8) {
        int8_t d = (int8_t)(cmd[7] - '0');
        if (d >= 1 && d <= 7) {
            DS3231_SetDateTime(ds3231_Date.year, ds3231_Date.month, ds3231_Date.date, (uint8_t)d, ds3231_Time.hour,
                               ds3231_Time.minutes, ds3231_Time.seconds);
            at_reply("OK\r\n");
        } else {
            at_reply("ERROR\r\n");
        }

        return;
    }

    /* AT+TIME? - query time */
    if (strcmp(cmd, "AT+TIME?") == 0) {
        char b[16];
        xsprintf(b, "%02d:%02d:%02d\n", ds3231_Time.hour, ds3231_Time.minutes, ds3231_Time.seconds);
        at_reply(b);
        at_reply("OK\r\n");
        return;
    }

    /* AT+DATE? - query date */
    if (strcmp(cmd, "AT+DATE?") == 0) {
        char b[16];
        xsprintf(b, "%02d.%02d.20%02d\n", ds3231_Date.date, ds3231_Date.month, ds3231_Date.year);
        at_reply(b);
        at_reply("OK\r\n");
        return;
    }

    /* AT+TEMP? - query temperature */
    if (strcmp(cmd, "AT+TEMP?") == 0) {
        char b[16];
        xsprintf(b, "%d.%d C\n", ds3231_Temp.temp_1, ds3231_Temp.temp_2);
        at_reply(b);
        at_reply("OK\r\n");
        return;
    }

    /* ATDP - "dial pulse": beep the buzzer when it's fitted */
    if (strncmp(cmd, "ATDP", 4) == 0) {
        /* buzzer_beep();  TODO when buzzer is soldered on TIM3 PWM */
        at_reply("OK\r\n");

        return;
    }

    /* unknown */
    at_reply("ERROR\r\n");
}

void UART_AT_Init(void) {
    LL_USART_EnableIT_RXNE(USART1);
}

void UART_AT_RxISR(void) {
    if (LL_USART_IsActiveFlag_RXNE(USART1)) {
        uint8_t c = LL_USART_ReceiveData8(USART1);
        uint8_t next = (uint8_t)((rxHead + 1) & (RX_BUF_SIZE - 1));

        if (next != rxTail) {
            rxBuf[rxHead] = c;
            rxHead = next;
        }
        /* if full - byte dropped (harmless for slow AT input) */
    }

    /* overrun guard: clear ORE or RX may stall after line noise */
    if (LL_USART_IsActiveFlag_ORE(USART1)) {
        LL_USART_ClearFlag_ORE(USART1);
    }
}

void UART_AT_Poll(void) {
    while (rxTail != rxHead) {
        uint8_t c = rxBuf[rxTail];
        rxTail = (uint8_t)((rxTail + 1) & (RX_BUF_SIZE - 1));

        if (echoEnabled) {
            uart_putc((char)c);
        }

        if (c == '\r' || c == '\n') {
            if (cmdLen > 0) {
                cmdBuf[cmdLen] = '\0';
                if (echoEnabled) {
                    at_reply("\r\n");
                }

                at_execute(cmdBuf);
                cmdLen = 0;
            }
        } else if (c == 8 || c == 127) { /* backspace / DEL */
            if (cmdLen > 0) {
                cmdLen--;
            }
        } else if (cmdLen < CMD_BUF_SIZE - 1) {
            cmdBuf[cmdLen++] = (char)c;
        }
        /* overflow: extra chars ignored until \r */
    }
}
