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
#define MODE_TIMEOUT_DATE  9
#define MODE_TIMEOUT_OTHER 5

#define VFD_BR_MAX      8
#define BREATH_STEP_MS  90    // ms per step; 90 × 16 = 1.45 s/cycle
#define ANIM_STEP_MS    60
#define NOT_DIGIT_VALUE 0xFF
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

static const uint8_t gamma_lut[] = {1, 1, 2, 3, 4, 5, 6, 8, 8, 6, 5, 4, 3, 2, 1, 1};

#define GAMMA_STEPS (sizeof(gamma_lut))

static uint8_t breathIdx = 0;
static uint32_t lastBreathTick = 0;

typedef struct {
    uint8_t segments;    // VFD_DIGIT_x / VFD_CHAR_x / 0
    uint8_t dot;
    uint8_t brightness;    // 0..VFD_BR_MAX; for digits always = VFD_BR_MAX
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

static uint8_t pwmPhase = 0;    // 0..VFD_BR_MAX-1, one step per frame

static uint8_t animationDigitCurrent[9];
static uint8_t animationDigitTarget[9];
static uint8_t isAnimationInProgress = 0;
static uint32_t lastAnimationTick = 0;

volatile uint8_t pendingMode = 0;
volatile uint8_t pendingTransition = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static inline void vfd_multiplex_tick(void);
static void build_frame(void);
static uint8_t breathing_level(void);
static void anim_start(uint8_t targetMode);
static void anim_step(void);
static uint8_t fwd_dist(uint8_t a, uint8_t b);
static void fill_target_digits(uint8_t mode, uint8_t *t);
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
                pendingMode = MODE_SHOW_DATE;
                pendingTransition = 1;
            }
            break;
        case MODE_SHOW_DATE:
            if (modeTimeout == MODE_TIMEOUT_DATE) {
                modeTimeout = 0;
                pendingMode = MODE_SHOW_TEMP_DAY;
                pendingTransition = 1;
            }
            break;
        case MODE_SHOW_TEMP_DAY:
            if (modeTimeout == MODE_TIMEOUT_OTHER) {
                modeTimeout = 0;
                pendingMode = MODE_SHOW_TIME;
                pendingTransition = 1;
            }
            break;
        }
    }
}

static uint8_t breathing_level(void) {
    uint32_t now = HAL_GetTick();

    if (now - lastBreathTick >= BREATH_STEP_MS) {
        lastBreathTick = now;
        if (++breathIdx >= GAMMA_STEPS)
            breathIdx = 0;
    }

    return gamma_lut[breathIdx];
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

static uint8_t fwd_dist(uint8_t a, uint8_t b) {
    return (uint8_t)((b + 10 - a) % 10);
}

static void anim_step(void) {
    if (!isAnimationInProgress) {
        return;
    }

    uint32_t now = HAL_GetTick();
    if (now - lastAnimationTick < ANIM_STEP_MS) {
        return;
    }

    lastAnimationTick = now;

    uint8_t anyMoving = 0;

    for (uint8_t i = 1; i <= 8; i++) {
        if (animationDigitTarget[i] == NOT_DIGIT_VALUE) {
            continue;
        }

        if (animationDigitCurrent[i] == animationDigitTarget[i]) {
            continue;
        }

        uint8_t up = fwd_dist(animationDigitCurrent[i], animationDigitTarget[i]);
        uint8_t down = (uint8_t)(10 - up);

        if (up <= down) {
            animationDigitCurrent[i] = (uint8_t)((animationDigitCurrent[i] + 1) % 10);
        } else {
            animationDigitCurrent[i] = (uint8_t)((animationDigitCurrent[i] + 9) % 10);
        }

        anyMoving = 1;
    }

    if (!anyMoving) {
        isAnimationInProgress = 0;
        currentMode = pendingMode;
    }
}

static void anim_start(uint8_t targetMode) {
    uint8_t src[9];

    fill_target_digits(currentMode, src);
    fill_target_digits(targetMode, animationDigitTarget);

    for (uint8_t i = 1; i <= 8; i++) {
        animationDigitCurrent[i] = (src[i] == NOT_DIGIT_VALUE) ? 0 : src[i];
    }

    isAnimationInProgress = 1;
    lastAnimationTick = HAL_GetTick();
}

static void fill_target_digits(uint8_t mode, uint8_t *t) {
    for (uint8_t i = 1; i <= 8; i++) {
        t[i] = NOT_DIGIT_VALUE;    // по умолчанию — не участвует
    }

    switch (mode) {
    case MODE_SHOW_TIME:
        t[8] = ds3231_Time.hour / 10;
        t[7] = ds3231_Time.hour % 10;

        t[5] = ds3231_Time.minutes / 10;
        t[4] = ds3231_Time.minutes % 10;

        t[2] = ds3231_Time.seconds / 10;
        t[1] = ds3231_Time.seconds % 10;
        break;

    case MODE_SHOW_DATE:
        t[8] = ds3231_Date.date / 10;
        t[7] = ds3231_Date.date % 10;
        t[6] = ds3231_Date.month / 10;
        t[5] = ds3231_Date.month % 10;
        t[4] = 2;
        t[3] = 0;
        t[2] = ds3231_Date.year / 10;
        t[1] = ds3231_Date.year % 10;
        break;

    case MODE_SHOW_TEMP_DAY:
        t[8] = ds3231_Temp.temp_1 / 10;
        t[7] = ds3231_Temp.temp_1 % 10;

        t[1] = ds3231_Date.day;
        break;
    }
}

static inline void vfd_multiplex_tick(void) {
    uint8_t seg = frame[curGrid].segments;
    uint8_t dot = frame[curGrid].dot;

    if (frame[curGrid].brightness <= pwmPhase) {
        seg = 0;
        dot = 0;
    }

    vfdBuf[2] = VFD_GRIDS[curGrid];
    vfdBuf[1] = seg;
    vfdBuf[0] = dot;

    VFD_LOAD_OFF;

    spi_send3();

    VFD_LOAD_ON;

    if (++curGrid > 8) {
        curGrid = 1;

        if (++pwmPhase >= VFD_BR_MAX) {
            pwmPhase = 0;
        }
    }
}

static void build_frame(void) {
    VfdCell f[9];
    for (uint8_t i = 1; i <= 8; i++) {
        f[i].segments = 0;
        f[i].dot = 0;
        f[i].brightness = VFD_BR_MAX;
    }

    if (isAnimationInProgress) {
        for (uint8_t i = 1; i <= 8; i++) {
            if (animationDigitTarget[i] != NOT_DIGIT_VALUE) {
                f[i].segments = VFD_DIGITS[animationDigitCurrent[i]];
            }
        }
    } else {
        uint8_t br = breathing_level();

        switch (currentMode) {
        case MODE_SHOW_TIME:
            f[8].segments = VFD_DIGITS[ds3231_Time.hour / 10];
            f[7].segments = VFD_DIGITS[ds3231_Time.hour % 10];
            f[6].segments = VFD_CHAR_MINUS;
            f[6].brightness = br;

            f[5].segments = VFD_DIGITS[ds3231_Time.minutes / 10];
            f[4].segments = VFD_DIGITS[ds3231_Time.minutes % 10];
            f[3].segments = VFD_CHAR_MINUS;
            f[3].brightness = br;

            f[2].segments = VFD_DIGITS[ds3231_Time.seconds / 10];
            f[1].segments = VFD_DIGITS[ds3231_Time.seconds % 10];
            break;
        case MODE_SHOW_DATE:
            f[8].segments = VFD_DIGITS[ds3231_Date.date / 10];
            f[7].segments = VFD_DIGITS[ds3231_Date.date % 10];

            f[7].dot = (blinkers ? 0 : 1);

            f[6].segments = VFD_DIGITS[ds3231_Date.month / 10];
            f[5].segments = VFD_DIGITS[ds3231_Date.month % 10];

            f[5].dot = (blinkers ? 0 : 1);

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
    //
    // Also see AT+DATE/AT+TIME/AT+DOW in uart_at.c

    DS3231_ReadData();

    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim14);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        if (!isAnimationInProgress && shouldReadDS3231) {    // во время анимации DS3231 не читаем
            shouldReadDS3231 = 0;
            DS3231_ReadData();
        }

        if (pendingTransition && !isAnimationInProgress) {
            pendingTransition = 0;
            anim_start(pendingMode);
        }

        anim_step();
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
