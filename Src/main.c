/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "ds3231.h"
#include "uart_at.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define VFD_LOAD_ON  HAL_GPIO_WritePin(VFD_LOAD_GPIO_Port, VFD_LOAD_Pin, GPIO_PIN_SET)
#define VFD_LOAD_OFF HAL_GPIO_WritePin(VFD_LOAD_GPIO_Port, VFD_LOAD_Pin, GPIO_PIN_RESET)

#define VFD_SEG_A 0b00000010
#define VFD_SEG_B 0b00000100
#define VFD_SEG_C 0b00001000
#define VFD_SEG_D 0b00010000
#define VFD_SEG_E 0b00100000
#define VFD_SEG_F 0b01000000
#define VFD_SEG_G 0b10000000

#define VFD_DIGIT_0 (VFD_SEG_A | VFD_SEG_B | VFD_SEG_C | VFD_SEG_D | VFD_SEG_E | VFD_SEG_F)
#define VFD_DIGIT_1 (VFD_SEG_B | VFD_SEG_C)
#define VFD_DIGIT_2 (VFD_SEG_A | VFD_SEG_B | VFD_SEG_D | VFD_SEG_E | VFD_SEG_G)
#define VFD_DIGIT_3 (VFD_SEG_A | VFD_SEG_B | VFD_SEG_C | VFD_SEG_D | VFD_SEG_G)
#define VFD_DIGIT_4 (VFD_SEG_B | VFD_SEG_C | VFD_SEG_F | VFD_SEG_G)
#define VFD_DIGIT_5 (VFD_SEG_A | VFD_SEG_C | VFD_SEG_D | VFD_SEG_F | VFD_SEG_G)
#define VFD_DIGIT_6 (VFD_SEG_A | VFD_SEG_C | VFD_SEG_D | VFD_SEG_E | VFD_SEG_F | VFD_SEG_G)
#define VFD_DIGIT_7 (VFD_SEG_A | VFD_SEG_B | VFD_SEG_C)
#define VFD_DIGIT_8 (VFD_SEG_A | VFD_SEG_B | VFD_SEG_C | VFD_SEG_D | VFD_SEG_E | VFD_SEG_F | VFD_SEG_G)
#define VFD_DIGIT_9 (VFD_SEG_A | VFD_SEG_B | VFD_SEG_C | VFD_SEG_D | VFD_SEG_F | VFD_SEG_G)

#define VFD_CHAR_MINUS  VFD_SEG_G
#define VFD_CHAR_DEGREE (VFD_SEG_A | VFD_SEG_B | VFD_SEG_F | VFD_SEG_G)
#define VFD_CHAR_C      (VFD_SEG_A | VFD_SEG_E | VFD_SEG_D | VFD_SEG_F)

#define MODE_SHOW_TIME     1
#define MODE_SHOW_DATE     2
#define MODE_SHOW_TEMP_DAY 3

#define RX_BUF_SIZE 64

// Settings modes - declared but not yet wired to UI.
// Kept here to show the planned state machine; see TODO in
// HAL_GPIO_EXTI_Callback for BTN_1 / BTN_2.
//
// Current firmware cycles BTN_3 only between MODE_SHOW_* (see below).
#define MODE_SETTINGS_TIME_HOURS       4
#define MODE_SETTINGS_TIME_MINUTES     5
#define MODE_SETTINGS_DATE_DAY         6
#define MODE_SETTINGS_DATE_MONTH       7
#define MODE_SETTINGS_DATE_YEAR        8
#define MODE_SETTINGS_DATE_DAY_OF_WEEK 9
#define MODE_SETTINGS_LAST_POSITION    MODE_SETTINGS_DATE_DAY_OF_WEEK

#define SELECTED_POSITION_FIRST  0
#define SELECTED_POSITION_SECOND 1
#define SELECTED_POSITION_THIRD  2
#define SELECTED_POSITION_LAST   SELECTED_POSITION_THIRD

#define MODE_TIMEOUT_TIME  35
#define MODE_TIMEOUT_DATE  7
#define MODE_TIMEOUT_OTHER 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static const uint8_t VFD_GRIDS[] = {
    0,
    0b00000001,    //	1
    0b00000010,    //	2
    0b00000100,    //	3
    0b00001000,    //	4
    0b00010000,    //	5
    0b01000000,    //	6
    0b00100000,    //	7
    0b10000000,    //	8
};

static const uint8_t VFD_DIGITS[] = {VFD_DIGIT_0, VFD_DIGIT_1, VFD_DIGIT_2, VFD_DIGIT_3, VFD_DIGIT_4,
                                     VFD_DIGIT_5, VFD_DIGIT_6, VFD_DIGIT_7, VFD_DIGIT_8, VFD_DIGIT_9};

typedef struct {
    uint8_t segments;    // VFD_DIGIT_x / VFD_CHAR_x / 0
    uint8_t dot;
} VfdCell;

volatile VfdCell frame[9];
volatile uint8_t curGrid = 1;

volatile uint8_t vfdBuf[3];

volatile uint8_t rxBuf[RX_BUF_SIZE];
volatile uint8_t rxHead = 0;
volatile uint8_t rxTail = 0;
volatile uint8_t echoEnabled = 1;    // ATE1/ATE0

volatile uint8_t selectedPos = SELECTED_POSITION_FIRST;
volatile uint8_t blinkers;
volatile uint8_t currentMode = MODE_SHOW_TIME;
volatile uint8_t modeTimeout = 0;
volatile uint8_t halfSeconds = 0;
volatile uint8_t shouldReadDS3231 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static inline void vfd_multiplex_tick(void);
static void build_frame(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    switch (GPIO_Pin) {
    case BTN_1_Pin:
        // TODO: select next position within current settings mode
        //       (see MODE_SETTINGS_* and selectedPos in this file).
        break;

    case BTN_2_Pin:
        // TODO: change value at selectedPos (increment / decrement
        //       depending on long/short press, with debounce).
        break;

    case BTN_3_Pin:
        currentMode++;

        // TODO: extend upper bound to MODE_SETTINGS_LAST_POSITION
        //       when settings UI (BTN_1 / BTN_2 handling) is implemented.
        if (currentMode > MODE_SHOW_TEMP_DAY) {
            currentMode = MODE_SHOW_TIME;
        }
        break;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM14) {
        vfd_multiplex_tick();

        return;
    }

    if (htim->Instance == TIM1) {
        blinkers ^= 1 << 1;
        modeTimeout++;
        halfSeconds++;

        if (halfSeconds == 2) {
            halfSeconds = 0;
            shouldReadDS3231 = 1;
        }

        switch (currentMode) {
        case MODE_SHOW_TIME:
            if (modeTimeout == MODE_TIMEOUT_TIME) {
                modeTimeout = 0;
                currentMode = MODE_SHOW_DATE;
            }

            break;

        case MODE_SHOW_DATE:
            if (modeTimeout == MODE_TIMEOUT_DATE) {
                modeTimeout = 0;
                currentMode = MODE_SHOW_TEMP_DAY;
            }

            break;

        case MODE_SHOW_TEMP_DAY:
            if (modeTimeout == MODE_TIMEOUT_OTHER) {
                modeTimeout = 0;
                currentMode = MODE_SHOW_TIME;
            }

            break;
        }
    }
}

static inline void spi_send3(void) {
    for (uint8_t i = 0; i < 3; i++) {
        while (!LL_SPI_IsActiveFlag_TXE(SPI1)) {
        }

        LL_SPI_TransmitData8(SPI1, vfdBuf[i]);
    }

    // wait for end before latch
    while (LL_SPI_IsActiveFlag_BSY(SPI1)) {
    }
}

static inline void vfd_multiplex_tick(void) {
    vfdBuf[2] = VFD_GRIDS[curGrid];
    vfdBuf[1] = frame[curGrid].segments;
    vfdBuf[0] = frame[curGrid].dot;

    VFD_LOAD_OFF;

    spi_send3();

    VFD_LOAD_ON;

    if (++curGrid > 8) {
        curGrid = 1;
    }
}

static void build_frame(void) {
    VfdCell f[9];

    for (uint8_t i = 1; i <= 8; i++) {
        f[i].segments = 0;
        f[i].dot = 0;
    }

    switch (currentMode) {
    case MODE_SHOW_TIME:
        f[8].segments = VFD_DIGITS[ds3231_Time.hour / 10];
        f[7].segments = VFD_DIGITS[ds3231_Time.hour % 10];

        f[6].segments = (blinkers) ? VFD_CHAR_MINUS : 0;

        f[5].segments = VFD_DIGITS[ds3231_Time.minutes / 10];
        f[4].segments = VFD_DIGITS[ds3231_Time.minutes % 10];

        f[3].segments = (blinkers) ? VFD_CHAR_MINUS : 0;

        f[2].segments = VFD_DIGITS[ds3231_Time.seconds / 10];
        f[1].segments = VFD_DIGITS[ds3231_Time.seconds % 10];
        break;

    case MODE_SHOW_DATE:
        f[8].segments = VFD_DIGITS[ds3231_Date.date / 10];
        f[7].segments = VFD_DIGITS[ds3231_Date.date % 10];

        f[7].dot = (blinkers) ? 0 : 1;

        f[6].segments = VFD_DIGITS[ds3231_Date.month / 10];
        f[5].segments = VFD_DIGITS[ds3231_Date.month % 10];

        f[5].dot = (blinkers) ? 0 : 1;
        f[4].segments = VFD_DIGITS[2];
        f[3].segments = VFD_DIGITS[0];

        f[2].segments = VFD_DIGITS[ds3231_Date.year / 10];
        f[1].segments = VFD_DIGITS[ds3231_Date.year % 10];
        break;

    case MODE_SHOW_TEMP_DAY:
        f[8].segments = VFD_DIGITS[ds3231_Temp.temp_1 / 10];
        f[7].segments = VFD_DIGITS[ds3231_Temp.temp_1 % 10];

        f[6].segments = VFD_CHAR_DEGREE;
        f[5].segments = VFD_CHAR_C;

        f[1].segments = VFD_DIGITS[ds3231_Date.day];
        break;
    }

    __disable_irq();

    for (uint8_t i = 1; i <= 8; i++) {
        frame[i] = f[i];
    }

    __enable_irq();
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_I2C1_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    MX_TIM1_Init();
    MX_TIM14_Init();
    /* USER CODE BEGIN 2 */
    UART_AT_Init();

    VFD_LOAD_OFF;

    // First-flash seeding of the RTC.
    //
    // Uncomment the DS3231_ReadData() line below, flash, let the device run once,
    // then comment it back and flash again. Otherwise the RTC
    // will be re-seeded on every reset.
    //
    // Arguments: year, month, date, day_of_week, hour, minutes, seconds
    //
    //     DS3231_SetDateTime(26, 6, 29, 1, 1, 36, 45);

    DS3231_ReadData();

    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim14);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        if (shouldReadDS3231) {
            DS3231_ReadData();
            shouldReadDS3231 = 0;
        }

        UART_AT_Poll();

        build_frame();
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
	 */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
