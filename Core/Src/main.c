/* USER CODE BEGIN Header */
//***************************************************************
//*   Company Name : Simatelex Manufactory Co., Ltd		*
//*   Project Name : 			*
//*   File Name    : main.c				 	*
//*   Author 	   : Yang Wen				        *
//*   Version      : T1.1					*
//*   MCU          : STM32G030C8T6			    	*
//*   Voltage      : 3.3V				   	*
//*   OSC          : On-chip 16MHz				*
//*   WDT          : ENABLE					*
//*   Start Date   : 7-Nov-2023				        *
//*   Last Update  : 04-Jan-2024			        *
//***************************************************************
//* ------------- Software Change List --------------------
//*********************************************************
//* First software version (T1.0)
//* Second software version (T1.1)
//*******************************************************************************************
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "constant.h"
#include "memory.h"
#include "TLC6983.h"
#include "BMA400.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim17;
DMA_HandleTypeDef hdma_tim17_up;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
extern const uint16_t PhaseAngle_50Hz_TAB[PHASE_ANGLE_STEPS];
extern const uint16_t PhaseAngle_60Hz_TAB[PHASE_ANGLE_STEPS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_IWDG_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void AC_Detect(void);
extern void Read_Key(void);
extern void Check_Nozzle(void);
extern void Check_Interlock(void);
extern void Check_Encoder(void);
extern void Check_Test(void);
extern void Do_Mode(void);
extern void Do_Startup_Mes(void);
extern void showhello(void);
#if (F_DEBUG_EN == 1)
extern void Uart_Debug(void);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/****************************************************************************
 * Turn on SYS_3.3V and SYS_5V
 * used by dot matrix, accelerometer, thermostat, AC Freq and Hall sensor
 *
 * There is a voltage drop on 3.3V and 5V when turning on the dot matrix
 * Lower the system freq to 1MHz before to avoid flash ECC errors
 * And also wait 20us after turning on before continuing.
 *
 **/
static void Voltage_Supply_Init(void)
{
    // use timer 14 to wait 20us after voltage is turned on
    TIM14->CR1 = 0; // stop timer
    TIM14->PSC = 0; // 1 MHz with low freq system clock
    TIM14->EGR |= TIM_EVENTSOURCE_UPDATE; // to load the new prescaler
    TIM14->CNT = 0;
    TIM14->ARR = 20 - 1;  // 20us
    TIM14->SR = 0;

    // lower system clock and turn voltage on
    SET_BIT(RCC->CR, RCC_HSI_DIV16); // lower system clock to 1 MHz
    VOLTAGE_SUPPLY_ON();
    // wait 20us
    TIM14->CR1 = TIM_CR1_CEN;
    while ((TIM14->SR & TIM_SR_UIF) == 0) ;
    CLEAR_BIT(RCC->CR, RCC_HSI_DIV128);  // restore system clock

    // restore timer 14 settings for AC Freq detection
    TIM14->CR1 = 0;  // stop timer
    TIM14->PSC = 15; // 1 MHz with normal freq system clock
    TIM14->EGR |= TIM_EVENTSOURCE_UPDATE; // to load the new prescaler
    TIM14->CNT = 0;
    TIM14->ARR = 200 - 1; // 200us
    TIM14->SR = 0;
}

void User_Init(void)
{
    F_AC_DETECT = AC_DETECT_NONE;
    F_INTERLOCK_CLOSE = 2;
    F_NOZZLE_CLOSE = 2;
    F_THERMOSTAT_CLOSE = 0;
    F_ENCODER_A_LOW = 2;
    F_ENCODER_B_LOW = 2;
    PID_Kp_CW = C_PID_KP_CW;
    PID_Ki_CW = C_PID_KI_CW;
    PID_Kd_CW = C_PID_KD_CW;
    PID_Kp_CCW = C_PID_KP_CCW;
    PID_Ki_CCW = C_PID_KI_CCW;
    PID_Kd_CCW = C_PID_KD_CCW;
    adjust_time = DEFAULT_BLEND_TIME;  // init blend time
    pour_time = DEFAULT_POUR_TIME;
    motor_phase_angle_table = (uint16_t*)PhaseAngle_50Hz_TAB;

#ifdef F_DEBUG_EN
    uart_tx_cnt = 0;
    uart_rx_cnt = 0;
    F_UART_RX = 0;
    F_UART_TX = 0;
#endif

    mode_cnt = 0;
    mode_sec_cnt = 0;
    breathing_zero_cnt = 0;
    mode = MODE_STARTUP_MES;
}


void Refresh_Control(void)
{
    if(F_MOTOR_RELAY == 1)
    {
        MOTOR_RELAY_ON();
    }
    else
    {
        MOTOR_RELAY_OFF();
    }

    if(F_MOTOR_CW == 1)
    {
        CW_RELAY_ON();
    }
    else
    {
        CW_RELAY_OFF();
    }

    if(F_MOTOR_CCW == 1)
    {
        CCW_RELAY_ON();
    }
    else
    {
        CCW_RELAY_OFF();
    }
    return;
}


//***********************************************************
//* AC frequence detection routine                         **
//* At each zero crossing check 200us counter hz_count,    **
//* then samp_50hz and samp_60hz counters are incremented  **
//* depending on the frequency detected.                   **
//* The final detection checks which of samp_50hz          **
//* and samp_60hz is bigger after a period of 500ms        **
//***********************************************************
void AC_Detect(void)
{
    if (F_AC_DETECT != AC_DETECT_NONE) return;

    ac_base_count++;
    if (ac_base_count >= T_05SEC)  // finish AC frequence detection within 0.5sec
    {
        ac_base_count = 0;
        if (samp_50hz >= 18&&samp_60hz < 21)
        {
            motor_phase_angle_table = (uint16_t*)PhaseAngle_50Hz_TAB; // AC frequency is 50hz
            half_ac_cyc = 9999; // 10 ms
            hz_count = 0;
            F_AC_DETECT = AC_DETECT_50HZ;
            HAL_TIM_Base_Stop_IT(&htim14);
            CLEAR_BIT(EXTI->FTSR1, EXTI_FTSR1_FT4); // disable AC Freq pin interrupt
            HAL_NVIC_DisableIRQ(AC_50_60HZ_EXTI_IRQn);
        }
        else if (samp_60hz >= 21&&samp_50hz < 18)
        {
            motor_phase_angle_table = (uint16_t*)PhaseAngle_60Hz_TAB; // AC frequency is 60hz
            half_ac_cyc = 8332; // 8.333 ms
            hz_count = 0;
            F_AC_DETECT = AC_DETECT_60HZ;
            HAL_TIM_Base_Stop_IT(&htim14);
            CLEAR_BIT(EXTI->FTSR1, EXTI_FTSR1_FT4); // disable AC Freq pin interrupt
            HAL_NVIC_DisableIRQ(AC_50_60HZ_EXTI_IRQn);
        }
        else  // default setting
        {
            samp_50hz = 0;
            samp_60hz = 0;
            hz_count = 0;
        }
    }
    return;
}

/*
 * Init Timers used for motor speed
 * calculation using Hall sensor
 **/
void Motor_Speed_Timer_Init(void)
{
    // Timer 3 is set to use Hall sensor pin as external clock
    // and send a trigger to Timer 1 every set number of turns (set in MotorPIDInit())
    TIM3->CR1 = 0; // stop timer
    TIM3->SMCR |= TIM_TS_TI1FP1 | TIM_SLAVEMODE_EXTERNAL1; // set CH1 (Hall sensor pin) as external clock
    TIM3->CCER |= TIM_TRIGGERPOLARITY_BOTHEDGE; // 1 motor turn has 6 edges
    TIM3->CR2 |= TIM_TRGO_UPDATE;  // set trigger output on overflow
    TIM3->PSC = 0;  // no divide prescaller

    // Timer 1 is set to run at 100000 Hz and capture the time
    // between triggers from Timer 3
    CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN);  // stop timer
    CLEAR_BIT(TIM1->CR1, TIM_CR1_CMS);  // clear CMS
    TIM1->CR1 = 0;                      // clear other bits
    TIM1->SMCR |= TIM_TS_ITR2 | TIM_SLAVEMODE_COMBINED_RESETTRIGGER;  // input trigger from Timer 3 resets timer
    TIM1->CCMR1 |= TIM_ICSELECTION_TRC;  // capture counter on trigger
    TIM1->CCER |= TIM_TRIGGERPOLARITY_RISING | TIM_CCER_CC1E;  // enable capture
    TIM1->PSC = 159; // 100000 Hz
    TIM1->ARR = 0xFFFF;  // set to max
    TIM1->CR1 |= TIM_CR1_OPM; // one pulse mode so if it overflows when the speed is too low the counter will stay at 0

    TIM3->CR1 |= TIM_CR1_CEN;  // start Timer 3
}

/*
 *  Disable unused interrupts
 *  so they don't wake up in sleep mode
 **/
void Sleep_Init(void)
{
    // disable lid/nozzle/button/encoder interrupts
    HAL_NVIC_DisableIRQ(ENCODER_CCW_EXTI_IRQn);
    CLEAR_BIT(EXTI->RTSR1, EXTI_RTSR1_RT0 | EXTI_RTSR1_RT2 | EXTI_RTSR1_RT15); // Interlock, Encoder CCW, Nozzle
    CLEAR_BIT(EXTI->FTSR1, EXTI_FTSR1_FT0 | EXTI_FTSR1_FT2 | EXTI_FTSR1_FT13 | EXTI_FTSR1_FT15); // Interlock, Encoder CCW, Button, Nozzle

    // disable other unused interrupts
    CLEAR_BIT(DMA1_Channel2->CCR, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);  // channel 2 is for dotmatrix
    CLEAR_BIT(DMA1_Channel3->CCR, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);  // channel 3 is for button led
    CLEAR_BIT(SPI2->CR2, SPI_CR2_ERRIE);  // spi2 is for dotmatrix
    CLEAR_BIT(EXTI->RTSR1, EXTI_RTSR1_RT1); // Thermostat pin, only used when motor is on
    HAL_NVIC_DisableIRQ(THERMOSTAT_EXTI_IRQn);

    // set breathing speed, period of one step is RCR * ARR / Freq
    TIM17->RCR = 30;  // 400 steps * 30 * 250 / 1MHz = 3s period

    // init led breathing DMA
    DMA1_Channel3->CCR = LL_DMA_PDATAALIGN_HALFWORD | DMA_CCR_DIR | DMA_CCR_CIRC | DMA_CCR_MINC;
    DMA1_Channel3->CPAR = (uint32_t)&(TIM17->CCR1); // destination

    // set periph clocks to stop in sleep mode
    // leave flash, dma and timer 17 for breathing led, timer 14 for watchdog refresh
    RCC->AHBSMENR = RCC_AHBSMENR_FLASHSMEN | RCC_AHBSMENR_DMA1SMEN;
    RCC->APBSMENR1 = 0;
    RCC->APBSMENR2 = RCC_APBSMENR2_TIM17SMEN | RCC_APBSMENR2_TIM14SMEN;
    RCC->IOPSMENR = 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_DMA_Init();
  MX_IWDG_Init();
  MX_SPI2_Init();
  MX_TIM17_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  Voltage_Supply_Init();
  User_Init();
  HAL_TIM_Base_Start_IT(&htim14);             // Timer 14 used for AC freq detection, Triac control and watchdog reset during sleep
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);  // Timer 17 used for button led PWM
  Motor_Speed_Timer_Init();                   // Timers 3 and 1 are used for speed measuring using Hall sensor
#ifdef F_DEBUG_EN
  HAL_UART_Receive_IT(&huart1, (uint8_t *)(&uart_rx_data), 1);
#endif
  InitialiseTLC6983();
  Sleep_Init(); // init some registers for sleep mode

  // fade-in startup message, do AC detect and check PCB test mode
  showhello();
  while (mode == MODE_STARTUP_MES)
  {
    CLR_WDT(); // Clear watch-dog
    if (F_TMBASE == 1)
    {
      F_TMBASE = 0;
      Check_Nozzle();
      Check_Interlock();
#ifdef F_PCBA_TEST
      Read_Key();
      Check_Test();
#endif
      Do_Startup_Mes();
      AC_Detect();
#if (F_DEBUG_EN == 1)
      Uart_Debug();
#endif
    }
  }
#ifndef F_VIBRATION_LIMIT_DISABLE
    bma400_init(); // need to be run at least 1ms after Voltage_Supply_Init()
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    CLR_WDT();    // Clear watch-dog
    if (F_TMBASE == 1)
    {
      F_TMBASE = 0;
      Check_Nozzle();
      Check_Interlock();  // function also calls Check_Thermostat()
      Check_Encoder();
      Read_Key();
      Do_Mode();
#if (F_DEBUG_EN == 1)
      Uart_Debug();
#endif
    }
    Refresh_Control();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_8);
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 159;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_TRC;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 15;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 199;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 15;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 249;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
#ifdef F_DEBUG_EN
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
#endif
  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DEBUG_3_Pin|MOTOR_CW_Pin|DEBUG_14_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, DEBUG_4_Pin|DEBUG_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin|DEBUG_8_Pin|DEBUG_9_Pin|DEBUG_13_Pin
                          |MOTOR_CCW_Pin|TRIAC_CONTROL_Pin|DIAL_ARROW_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DEBUG_10_Pin|EEPROM_SCL_Pin|EEPROM_SDA_Pin|MOTOR_RELAY_Pin
                          |PROCESSING_LED_Pin|VOLTAGE_SUPPLY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DEBUG_16_Pin|NOZZLE_LED_Pin|DEBUG_17_Pin|DISPENSING_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ON_OFF_BUTTON_Pin */
  GPIO_InitStruct.Pin = ON_OFF_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ON_OFF_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DEBUG_3_Pin MOTOR_CW_Pin DEBUG_14_Pin */
  GPIO_InitStruct.Pin = DEBUG_3_Pin|MOTOR_CW_Pin|DEBUG_14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : NOZZLE_INPUT_Pin */
  GPIO_InitStruct.Pin = NOZZLE_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NOZZLE_INPUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DEBUG_4_Pin DEBUG_5_Pin */
  GPIO_InitStruct.Pin = DEBUG_4_Pin|DEBUG_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : ENCODER_CW_Pin ACCEL_INT_Pin */
  GPIO_InitStruct.Pin = ENCODER_CW_Pin|ACCEL_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_CCW_Pin */
  GPIO_InitStruct.Pin = ENCODER_CCW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENCODER_CCW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AC_50_60HZ_Pin */
  GPIO_InitStruct.Pin = AC_50_60HZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AC_50_60HZ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DEBUG_8_Pin DEBUG_9_Pin DEBUG_13_Pin MOTOR_CCW_Pin
                           TRIAC_CONTROL_Pin DIAL_ARROW_LED_Pin */
  GPIO_InitStruct.Pin = DEBUG_8_Pin|DEBUG_9_Pin|DEBUG_13_Pin|MOTOR_CCW_Pin
                          |TRIAC_CONTROL_Pin|DIAL_ARROW_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : INTERLOCK_Pin */
  GPIO_InitStruct.Pin = INTERLOCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INTERLOCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : THERMOSTAT_Pin */
  GPIO_InitStruct.Pin = THERMOSTAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(THERMOSTAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DEBUG_10_Pin EEPROM_SCL_Pin EEPROM_SDA_Pin MOTOR_RELAY_Pin
                           PROCESSING_LED_Pin VOLTAGE_SUPPLY_Pin */
  GPIO_InitStruct.Pin = DEBUG_10_Pin|EEPROM_SCL_Pin|EEPROM_SDA_Pin|MOTOR_RELAY_Pin
                          |PROCESSING_LED_Pin|VOLTAGE_SUPPLY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DISPLAY_SCLK_Pin */
  GPIO_InitStruct.Pin = DISPLAY_SCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(DISPLAY_SCLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DEBUG_16_Pin NOZZLE_LED_Pin DEBUG_17_Pin DISPENSING_LED_Pin */
  GPIO_InitStruct.Pin = DEBUG_16_Pin|NOZZLE_LED_Pin|DEBUG_17_Pin|DISPENSING_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
 *  Wake up if lid/nozzle or button are closed
 *  otherwise go back to sleep after ISR return
 **/
void Sleep_Wake_Up(void)
{
    if (TIM14->CR1 & TIM_CR1_CEN && TIM14->PSC == 999) // timer is in sleep wake-up config
    {
        CLR_WDT();
        TIM14->CNT = 0;            // use timer 14 to wait 2ms
        while (TIM14->CNT < 2) ;   // to give time for GPIO registers to be ready
    }
#ifdef F_FW_VERSION_DISPLAY
    if (NOZZLE_PIN_STATUS() == 0 || INTERLOCK_PIN_STATUS() == 1 || ONOFF_KEY_PIN_STATUS() == 1)
#else
    if (NOZZLE_PIN_STATUS() == 0 || INTERLOCK_PIN_STATUS() == 1)
#endif
    {
        CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPONEXIT_Msk);
    }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == AC_50_60HZ_Pin)  // 50/60Hz interrupt
    {
        if (F_PASS_GUARD == 1) // pass guard time => this is valid zero crossing interrupt
        {
            F_PASS_GUARD = 0;
            ac_guard_cnt = 0;

            if (F_AC_DETECT != AC_DETECT_NONE)
            {
                if (F_MOTOR_TRIAC == 1)
                {
                    pulse_step = 0;
                    Motor_PhaseAngle = Motor_PhaseAngle_Cmd; // only apply new phase angle values
                    Triac_Pulse_Time = Triac_Pulse_Time_Cmd; // on begining of the cycle
                    TIM14->ARR = Motor_PhaseAngle; // set reload value
                    TIM14->SR = 0; // Clear the pending interrupt flag
                    TIM14->CNT = 0; // Reset the reload counter
                    TIM14->DIER = TIM_IT_UPDATE; // Enable the timer interrupt
                    TIM14->CR1 = TIM_CR1_CEN; // start timer
                    hz_count++;  // used to detect if AC signal is lost
                }
            }
            else
            {
                if (hz_count < 91)         // 91 * 0.2=18.2ms -> 55Hz
                {
                    if (hz_count >= 77)    // 77*0.2ms=15.4ms -> 65Hz
                    {
                        samp_60hz++;
                    }
                }
                else if (hz_count <= 110)  // 110*0.2ms=22ms -> 45.5Hz
                {
                    samp_50hz++;
                }
                hz_count = 0;
            }
        }
    }
    else if (GPIO_Pin == INTERLOCK_Pin || GPIO_Pin == NOZZLE_INPUT_Pin || GPIO_Pin == ENCODER_CCW_Pin || GPIO_Pin == ON_OFF_BUTTON_Pin)
    {
        Sleep_Wake_Up();
    }
}


void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == THERMOSTAT_Pin)
    {
        thermostat_pulse_cnt++;
    }
    else if (GPIO_Pin == INTERLOCK_Pin || GPIO_Pin == NOZZLE_INPUT_Pin || GPIO_Pin == ENCODER_CCW_Pin)
    {
        Sleep_Wake_Up();
    }
}


void HAL_SYSTICK_Callback(void)
{
    ms_cnt++;
    if (ms_cnt == 4)
    {
        F_TMBASE = 1;
        check_thermostat_cnt++;
        PID_Time++;
    }
    else if ((ms_cnt == 2 || ms_cnt == 6) && TLC6983Enabled && SRAMbuffer0Complete == 0 && hspi2.hdmatx->Instance->CNDTR == 0 && (SPI2->SR & SPI_SR_FTLVL) == 0)
    {
        dotmatrix_brighness_update(); // update dot matrix brightness every 4ms, but not at the same time as SRAM/Vsync
    }
    else if (ms_cnt >= 8)
    {
        ms_cnt = 0;
        F_TMBASE = 1;
        check_thermostat_cnt++;
        PID_Time++;
        if (TLC6983Enabled && hspi2.hdmatx->Instance->CNDTR == 0 && (SPI2->SR & SPI_SR_FTLVL) == 0)
        {
            // SPI TX DMA ready to be reloaded
            // reload DMA manually
            if (SRAMbuffer0Complete)
            {
                CLEAR_BIT(hspi2.hdmatx->Instance->CCR, DMA_CCR_EN);
                CLEAR_BIT(SPI2->CR2, SPI_CR2_TXDMAEN);
                hspi2.hdmatx->Instance->CNDTR = SRAMbuffer0Length;
                hspi2.hdmatx->Instance->CMAR = (uint32_t)SRAMbuffer0;
                SET_BIT(hspi2.hdmatx->Instance->CCR, DMA_CCR_EN);
                SET_BIT(SPI2->CR2, SPI_CR2_TXDMAEN);
                SRAMbuffer0Complete--;
            }
            else
            {
                CLEAR_BIT(hspi2.hdmatx->Instance->CCR, DMA_CCR_EN);
                CLEAR_BIT(SPI2->CR2, SPI_CR2_TXDMAEN);
                hspi2.hdmatx->Instance->CNDTR = VSyncBufferLength;
                hspi2.hdmatx->Instance->CMAR = (uint32_t)VsyncBuffer;
                SET_BIT(hspi2.hdmatx->Instance->CCR, DMA_CCR_EN);
                SET_BIT(SPI2->CR2, SPI_CR2_TXDMAEN);
                VSyncBufferSent = 1;
            }
        }
    }
    if (F_PASS_GUARD == 0)
    {
        ac_guard_cnt++;
        if (ac_guard_cnt >= 14)   // guard time: 1ms*14=14ms
        {
            ac_guard_cnt = 0;
            F_PASS_GUARD = 1;  // set pass guard time flag
        }
    }
}

/*
 * Timer 14 used for AC Frequency detection
 * then for Triac control.
 * This function is called from TIM14_IRQHandler()
 **/
void Timer14_Elapsed(void)
{
    if (TIM14->SR & TIM_FLAG_UPDATE)
    {
        TIM14->SR = 0; // clear interrupt flag
        if (F_MOTOR_TRIAC == 1)  // Triac control
        {
            switch (pulse_step)
            {
            case 0:  // 1st pulse Triac after phase angle delay on first half period
            case 2:  // 2nd pulse Triac after phase angle delay on second half period
                MOTOR_TRIAC_ON();
                TIM14->ARR = Triac_Pulse_Time;
                pulse_step++;
                break;
            case 1:  // end triac first pulse and wait for the next pulse on the second half period
                MOTOR_TRIAC_OFF();
                TIM14->ARR = (half_ac_cyc - (Motor_PhaseAngle + Triac_Pulse_Time) + Motor_PhaseAngle_Cmd);
                Motor_PhaseAngle = Motor_PhaseAngle_Cmd; // apply new phase angle values
                Triac_Pulse_Time = Triac_Pulse_Time_Cmd; // for next cycle
                pulse_step++;
                break;
            case 3:  // end triac second pulse and stop timer
                MOTOR_TRIAC_OFF();
                TIM14->DIER = 0;  // disable interrupt
                TIM14->CR1 = 0;   // stop timer
                break;
            }
        }
        else if (F_AC_DETECT == AC_DETECT_NONE)  // 200us timer for AC freq detection
        {
            hz_count++;
        }
        else if (mode == MODE_SLEEP)  // refresh watchdog in sleep mode
        {
            CLR_WDT();
            // check lid and nozzle status if sleep was not caused by timeout
            if (timed_out_sleep == 0 && (NOZZLE_PIN_STATUS() == 0 || INTERLOCK_PIN_STATUS() == 1))
            {
                CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPONEXIT_Msk);
            }
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
