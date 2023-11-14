/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023, Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#ifndef BOARD_H_
#define BOARD_H_

#ifdef __cplusplus
extern "C"
{
#endif

// LED GREEN
#define LED_PORT GPIOC
#define LED_PIN GPIO_PIN_7
#define LED_STATE_ON 1

// BUTTON
#define BUTTON_PORT GPIOC
#define BUTTON_PIN GPIO_PIN_13
#define BUTTON_STATE_ACTIVE 1

// UART Enable for STLink VCOM
#define UART_DEV USART1
#define UART_CLK_EN __HAL_RCC_USART1_CLK_ENABLE
#define UART_GPIO_PORT GPIOA
#define UART_GPIO_AF GPIO_AF7_USART1
#define UART_TX_PIN GPIO_PIN_9
#define UART_RX_PIN GPIO_PIN_10

/* ====================================================== SPARK FIRMWARE CODE ====================================================== */
/** @brief The Quasar's system clock selection.
 */
typedef enum quasar_clk_freq {
    /*! System clock at 144 MHz (USB) */
    QUASAR_CLK_144MHZ = 144000000,
    /*! System clock at 163.84 MHz */
    QUASAR_CLK_163_84MHZ = 163840000,
    /*! System clock at 160 MHz */
    QUASAR_CLK_160MHZ = 160000000,
} quasar_clk_freq_t;

/* Voltage scale clock frequency limit */
/*! High performance range (160 MHz). */
#define RANGE1_HIGH_PERFORMANCE_FREQ_LIMIT        160000000
/*! Medium-high performance range (110 MHz). */
#define RANGE2_MEDIUM_HIGH_PERFORMANCE_FREQ_LIMIT 110000000
/*! Medium-low performance range (55 MHz). */
#define RANGE3_MEDIUM_LOW_PERFORMANCE_FREQ_LIMIT  55000000
/*! Low-power performance range (25 MHz). */
#define RANGE4_LOW_POWER_PERFORMANCE_FREQ_LIMIT   25000000

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static void quasar_clock_init(quasar_clk_freq_t quasar_clk_freq);
static uint32_t quasar_clock_get_system_clock_freq(void);
static void config_clock_freq(RCC_OscInitTypeDef *RCC_OscInitStruct,
                              RCC_ClkInitTypeDef *RCC_ClkInitStruct,
                              quasar_clk_freq_t quasar_clk_freq);
static void set_oscillator_pll_144mhz(RCC_OscInitTypeDef *RCC_OscInitStruct);
static void set_oscillator_pll_163_84mhz(RCC_OscInitTypeDef *RCC_OscInitStruct);
static void set_oscillator_pll_160mhz(RCC_OscInitTypeDef *RCC_OscInitStruct);
static void set_system_clock_pll(RCC_ClkInitTypeDef *RCC_ClkInitStruct);
static void system_clock_init(RCC_OscInitTypeDef *RCC_OscInitStruct,
                              RCC_ClkInitTypeDef *RCC_ClkInitStruct,
                              uint32_t flash_latency);
static void update_systick(void);
static uint32_t get_voltage_scale(quasar_clk_freq_t quasar_clk_freq);
static uint32_t get_flash_latency(uint32_t voltage_scale, quasar_clk_freq_t quasar_clk_freq);
/* ================================================================================================================================= */

//--------------------------------------------------------------------+
// RCC Clock
//--------------------------------------------------------------------+

static void SystemClock_Config(void) {

  quasar_clock_init(QUASAR_CLK_144MHZ);
  // RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  // RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  // __HAL_RCC_PWR_CLK_ENABLE();
  // HAL_PWREx_EnableVddA();

  // /** Configure the main internal regulator output voltage
  // */
  // if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
  //   Error_Handler();
  // }

  // /** Initializes the CPU, AHB and APB buses clocks
  // */
  // RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
  // RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  // RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  // RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  // RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  // RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  // RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  // RCC_OscInitStruct.PLL.PLLM = 1;
  // RCC_OscInitStruct.PLL.PLLN = 20;
  // RCC_OscInitStruct.PLL.PLLP = 8;
  // RCC_OscInitStruct.PLL.PLLQ = 2;
  // RCC_OscInitStruct.PLL.PLLR = 2;
  // RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  // RCC_OscInitStruct.PLL.PLLFRACN = 0;
  // if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
  //   Error_Handler();
  // }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  // RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
  //                               | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
  //                               | RCC_CLOCKTYPE_PCLK3;
  // RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  // RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  // RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  // RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  // RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  // HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);

  // USB Clock
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  RCC_PeriphCLKInitTypeDef usb_clk_init = { 0};
  usb_clk_init.PeriphClockSelection = RCC_PERIPHCLK_USBPHY;
  // usb_clk_init.UsbPhyClockSelection = RCC_USBPHYCLKSOURCE_HSE;
  usb_clk_init.UsbPhyClockSelection = RCC_USBPHYCLKSOURCE_PLL1;
  if (HAL_RCCEx_PeriphCLKConfig(&usb_clk_init) != HAL_OK) {
    Error_Handler();
  }

  /** Set the OTG PHY reference clock selection
  */
  HAL_SYSCFG_SetOTGPHYReferenceClockSelection(SYSCFG_OTG_HS_PHY_CLK_SELECT_1);

  // USART clock
  RCC_PeriphCLKInitTypeDef uart_clk_init = { 0};
  uart_clk_init.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  uart_clk_init.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&uart_clk_init) != HAL_OK) {
    Error_Handler();
  }
}

static void SystemPower_Config(void) {
  HAL_PWREx_EnableVddIO2();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK) {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}


/* ====================================================== SPARK FIRMWARE CODE ====================================================== */
static void quasar_clock_init(quasar_clk_freq_t quasar_clk_freq)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    HAL_StatusTypeDef hal_status;

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_EnableVddA();
    // __HAL_RCC_GPDMA1_CLK_ENABLE();

    /* Update the SystemCoreClock global variable. */
    SystemCoreClock = HAL_RCC_GetSysClockFreq() >> AHBPrescTable[(RCC->CFGR2 & RCC_CFGR2_HPRE) >> RCC_CFGR2_HPRE_Pos];

    /* Use systick as time base source and configure 1ms tick (default clock after Reset is HSI). */
    HAL_InitTick(TICK_INT_PRIORITY);

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    uint32_t voltage_scale = 0;
    uint32_t flash_latency = 0;

    /* Calculate and set the voltage scale depending on the clock frequency. */
    voltage_scale = get_voltage_scale(quasar_clk_freq);
    hal_status = HAL_PWREx_ControlVoltageScaling(voltage_scale);
    if (hal_status != HAL_OK) {
        while (1);
    }

    /* Calculate the flash latency. */
    flash_latency = get_flash_latency(voltage_scale, quasar_clk_freq);

    config_clock_freq(&RCC_OscInitStruct, &RCC_ClkInitStruct, quasar_clk_freq);
    system_clock_init(&RCC_OscInitStruct, &RCC_ClkInitStruct, flash_latency);

    update_systick();
}

static uint32_t quasar_clock_get_system_clock_freq(void)
{
    /* Update Core Clock value */
    SystemCoreClockUpdate();

    return SystemCoreClock;
}

/** @brief Select clock initializations depending on the frequency.
 *
 *  @param[in] RCC_OscInitStruct  RCC Oscillator configuration structure definition.
 *  @param[in] RCC_ClkInitStruct  RCC System/AHB/APB clock config structure definition.
 *  @param[in] quasar_clk_freq    Clock frequency selection.
 */
static void config_clock_freq(RCC_OscInitTypeDef *RCC_OscInitStruct,
                              RCC_ClkInitTypeDef *RCC_ClkInitStruct,
                              quasar_clk_freq_t quasar_clk_freq)
{
    switch (quasar_clk_freq) {
    case QUASAR_CLK_144MHZ:
        set_oscillator_pll_144mhz(RCC_OscInitStruct);
        set_system_clock_pll(RCC_ClkInitStruct);
        break;
    case QUASAR_CLK_163_84MHZ:
        set_oscillator_pll_163_84mhz(RCC_OscInitStruct);
        set_system_clock_pll(RCC_ClkInitStruct);
        break;
    case QUASAR_CLK_160MHZ:
        set_oscillator_pll_160mhz(RCC_OscInitStruct);
        set_system_clock_pll(RCC_ClkInitStruct);
        break;
    default:
        while (1);
        break;
    }
}

/** @brief Initialize the PLL clock to 144 MHz.
 *
 *  The HSE was changed to 24.576 MHz on the nucleo board. Normal value should be 12.288 MHz.
 *
 *  @param[in] RCC_OscInitStruct  RCC Oscillator configuration structure definition.
 */
static void set_oscillator_pll_144mhz(RCC_OscInitTypeDef *RCC_OscInitStruct)
{
    RCC_OscInitStruct->OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct->HSEState = RCC_HSE_ON;
    RCC_OscInitStruct->HSIState = RCC_HSI_ON;
    RCC_OscInitStruct->PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct->PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct->PLL.PLLMBOOST = RCC_PLLMBOOST_DIV2;
    RCC_OscInitStruct->PLL.PLLM = 2;
    RCC_OscInitStruct->PLL.PLLN = 11;
    RCC_OscInitStruct->PLL.PLLP = 9;
    RCC_OscInitStruct->PLL.PLLQ = 2;
    RCC_OscInitStruct->PLL.PLLR = 1;
    RCC_OscInitStruct->PLL.PLLRGE = RCC_PLLVCIRANGE_1;
    RCC_OscInitStruct->PLL.PLLFRACN = 5888;
}

/** @brief Initialize the PLL clock to 163.84 MHz.
 *
 *  The HSE was changed to 24.576 MHz on the nucleo board. Normal value should be 12.288 MHz.
 *
 *  @param[in] RCC_OscInitStruct  RCC Oscillator configuration structure definition.
 */
static void set_oscillator_pll_163_84mhz(RCC_OscInitTypeDef *RCC_OscInitStruct)
{
    RCC_OscInitStruct->OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct->HSEState = RCC_HSE_ON;
    RCC_OscInitStruct->HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct->PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct->PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct->PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
    RCC_OscInitStruct->PLL.PLLM = 6;
    RCC_OscInitStruct->PLL.PLLN = 40;
    RCC_OscInitStruct->PLL.PLLP = 2;
    RCC_OscInitStruct->PLL.PLLQ = 1;
    RCC_OscInitStruct->PLL.PLLR = 1;
    RCC_OscInitStruct->PLL.PLLRGE = RCC_PLLVCIRANGE_0;
    RCC_OscInitStruct->PLL.PLLFRACN = 0;
}

/** @brief Initialize the PLL clock to 160 MHz.
 *
 *  The HSE was changed to 24.576 MHz on the nucleo board. Normal value should be 12.288 MHz.
 *
 *  @param[in] RCC_OscInitStruct  RCC Oscillator configuration structure definition.
 */
static void set_oscillator_pll_160mhz(RCC_OscInitTypeDef *RCC_OscInitStruct)
{
    RCC_OscInitStruct->OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct->HSEState = RCC_HSE_ON;
    RCC_OscInitStruct->HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct->PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct->PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct->PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
    RCC_OscInitStruct->PLL.PLLM = 6;
    RCC_OscInitStruct->PLL.PLLN = 39;
    RCC_OscInitStruct->PLL.PLLP = 2;
    RCC_OscInitStruct->PLL.PLLQ = 1;
    RCC_OscInitStruct->PLL.PLLR = 1;
    RCC_OscInitStruct->PLL.PLLRGE = RCC_PLLVCIRANGE_1;
    RCC_OscInitStruct->PLL.PLLFRACN = 0;
}

/** @brief Initializes the CPU, AHB and APB busses clocks.
 *
 *  @param[in] RCC_ClkInitStruct  RCC System/AHB/APB clock config structure definition.
 */
static void set_system_clock_pll(RCC_ClkInitTypeDef *RCC_ClkInitStruct)
{
    RCC_ClkInitStruct->ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                |RCC_CLOCKTYPE_PCLK3;
    RCC_ClkInitStruct->SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct->AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct->APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct->APB2CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct->APB3CLKDivider = RCC_HCLK_DIV1;
}

/** @brief Configure the oscillators and clock.
 *
 *  @param[in] RCC_OscInitStruct  Oscillator configuration structure definition.
 *  @param[in] RCC_ClkInitStruct  Clock configuration structure definition.
 *  @param[in] flash_latency      Flash latency value to correctly read data from FLASH memory.
 */
static void system_clock_init(RCC_OscInitTypeDef *RCC_OscInitStruct,
                              RCC_ClkInitTypeDef *RCC_ClkInitStruct,
                              uint32_t flash_latency)
{
    if (HAL_RCC_OscConfig(RCC_OscInitStruct) != HAL_OK) {
        while (1);
    }
    if (HAL_RCC_ClockConfig(RCC_ClkInitStruct, flash_latency) != HAL_OK) {
        while (1);
    }
}

/** @brief Adjust the systick with the clock frequency.
 */
static void update_systick(void)
{
    /* Generate a tick every 1 ms. */
    HAL_SYSTICK_Config(quasar_clock_get_system_clock_freq() / 1000);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

/** @brief Get the voltage scale depending on the frequency.
 *
 *  @param[in] quasar_clk_freq  Current board frequency.
 *  @return Voltage scale.
 */
static uint32_t get_voltage_scale(quasar_clk_freq_t quasar_clk_freq)
{
    uint32_t voltage_scale = 0;

    if (quasar_clk_freq < RANGE4_LOW_POWER_PERFORMANCE_FREQ_LIMIT) {
        voltage_scale = PWR_REGULATOR_VOLTAGE_SCALE4;
    } else if (quasar_clk_freq <= RANGE3_MEDIUM_LOW_PERFORMANCE_FREQ_LIMIT) {
        voltage_scale = PWR_REGULATOR_VOLTAGE_SCALE3;
    } else if (quasar_clk_freq <= RANGE2_MEDIUM_HIGH_PERFORMANCE_FREQ_LIMIT) {
        voltage_scale = RANGE2_MEDIUM_HIGH_PERFORMANCE_FREQ_LIMIT;
    } else if (quasar_clk_freq <= RANGE1_HIGH_PERFORMANCE_FREQ_LIMIT) {
        voltage_scale = PWR_REGULATOR_VOLTAGE_SCALE1;
    } else if (quasar_clk_freq > RANGE1_HIGH_PERFORMANCE_FREQ_LIMIT) {
        /* >160 Mhz (not recommended). */
        voltage_scale = PWR_REGULATOR_VOLTAGE_SCALE1;
    }

    return voltage_scale;
}

/** @brief Set the flash latency from the voltage scale and HCLK.
 *
 *  The values are taken from the datasheet.
 *
 *  @note It is assumed that the AHB prescaler is set to 1.
 *
 *  @param[in] voltage_scale    The selected voltage scale.
 *  @param[in] quasar_clk_freq  The selected clock frequency.
 *  @return The flash latency.
 */
static uint32_t get_flash_latency(uint32_t voltage_scale, quasar_clk_freq_t quasar_clk_freq)
{
    uint32_t flash_latency = 0;

    switch (voltage_scale) {
    case PWR_REGULATOR_VOLTAGE_SCALE1:
        if (quasar_clk_freq <= 32000000) {
            flash_latency = FLASH_LATENCY_0;
        } else if (quasar_clk_freq <= 64000000) {
            flash_latency = FLASH_LATENCY_1;
        } else if (quasar_clk_freq <= 96000000) {
            flash_latency = FLASH_LATENCY_2;
        } else if (quasar_clk_freq <= 128000000) {
            flash_latency = FLASH_LATENCY_3;
        } else if (quasar_clk_freq <= 160000000) {
            flash_latency = FLASH_LATENCY_4;
        } else if (quasar_clk_freq > 160000000) {
            /* >160Mhz (not recommended). */
            flash_latency = FLASH_LATENCY_4;
        }
        break;
    case PWR_REGULATOR_VOLTAGE_SCALE2:
        if (quasar_clk_freq <= 30000000) {
            flash_latency = FLASH_LATENCY_0;
        } else if (quasar_clk_freq <= 60000000) {
            flash_latency = FLASH_LATENCY_1;
        } else if (quasar_clk_freq <= 90000000) {
            flash_latency = FLASH_LATENCY_2;
        } else if (quasar_clk_freq <= 11000000) {
            flash_latency = FLASH_LATENCY_3;
        }
        break;
    case PWR_REGULATOR_VOLTAGE_SCALE3:
        if (quasar_clk_freq <= 24000000) {
            flash_latency = FLASH_LATENCY_0;
        } else if (quasar_clk_freq <= 48000000) {
            flash_latency = FLASH_LATENCY_1;
        } else if (quasar_clk_freq <= 55000000) {
            flash_latency = FLASH_LATENCY_2;
        }
        break;
    case PWR_REGULATOR_VOLTAGE_SCALE4:
        if (quasar_clk_freq <= 12000000) {
            flash_latency = FLASH_LATENCY_0;
        } else if (quasar_clk_freq <= 25000000) {
            flash_latency = FLASH_LATENCY_1;
        }
        break;
    }

    return flash_latency;
}

/* ================================================================================================================================= */


#ifdef __cplusplus
}
#endif

#endif /* BOARD_H_ */
