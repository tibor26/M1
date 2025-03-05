#include "stm32g0xx_hal.h"
#include "constant.h"
#include "memory.h"
#include "TLC6983.h"
#include "BMA400.h"
//=========================================================
void Do_Mode(void);
void Do_Startup_Mes(void);
void Go_To_Sleep(void);
void Go_To_Sleep_Timeout(void);
void Do_Sleep(void);
void Go_To_Adjust_Time(void);
void Do_Adjust_Time(void);
void Go_To_Blending(void);
void Do_Blending(void);
void Go_To_Spinning(void);
void Do_Spinning(void);
void Go_To_Error(uint8_t error_mode);
void Do_Error(void);
#ifdef F_FW_VERSION_DISPLAY
void Do_Firmware_Version(void);
#endif
uint8_t Dotmatrix_Breathe(void);
void Check_Timeout(void);

extern void MotorPIDControl(uint16_t TargetSpeed);
uint8_t Check_Motor_Speed(uint8_t delay_check, uint16_t speed_low, uint16_t speed_high, uint16_t speed_max);
extern void MotorPIDInit(int32_t kp, int32_t ki, int32_t kd, uint16_t output_init, uint8_t speed_period);
extern void Set_Triac_Off(void);
extern void clearDisplay(void);
extern void showTimeOnly(uint8_t thisCount);
extern void showCLOSE(void);
extern void showOPEN(void);
extern void showENJOY(void);
extern void showERROR(void);
extern void showLID(void);
extern void showSPOUT(void);
extern void showSPIN(void);
extern void showONLY(void);
extern void showSET(void);
extern void showBLEND(void);
extern void showTIME(void);
extern void showTHERM(void);
extern void showACHz(void);
extern void showACCEL(void);
extern void showString(char *thisString, uint8_t autoCenter, uint8_t fixedFontWidth);
extern void Do_Pcba_Test(void);
extern void Motor_Speed_Timer_Init(void);
extern void vibration_controller_init(uint16_t output_init);
extern uint16_t vibration_controller(uint16_t speed_target);

extern IWDG_HandleTypeDef hiwdg;


/* exp(sin(x)) breathing curve
 * Python:
 * for i in range(LED_BUTTON_SLEEP_STEPS):
 *     v = int( (math.exp(math.sin( (i/100+3)*(math.pi/2) ) ) - 1/math.e) * (150/(math.e-1/math.e)) )
 **/
static const uint8_t sleep_breathing_table[LED_BUTTON_SLEEP_STEPS] = {
  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   4,   4,   4,
  4,   5,   5,   5,   6,   6,   6,   6,   7,   7,   7,   8,   8,   9,   9,   9,  10,  10,  11,  11,  11,  12,  12,  13,  13,  14,  14,  15,  15,  16,  17,  17,  18,  18,  19,  20,  20,  21,  22,  22,
 23,  24,  24,  25,  26,  27,  27,  28,  29,  30,  31,  31,  32,  33,  34,  35,  36,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  51,  52,  53,  54,  55,  57,  58,  59,  60,  62,
 63,  64,  66,  67,  68,  70,  71,  72,  74,  75,  77,  78,  79,  81,  82,  84,  85,  87,  88,  89,  91,  92,  94,  95,  97,  98, 100, 101, 103, 104, 105, 107, 108, 110, 111, 113, 114, 115, 117, 118,
119, 121, 122, 123, 124, 126, 127, 128, 129, 130, 132, 133, 134, 135, 136, 137, 138, 139, 140, 140, 141, 142, 143, 143, 144, 145, 145, 146, 146, 147, 147, 148, 148, 148, 149, 149, 149, 149, 149, 149,
150, 149, 149, 149, 149, 149, 149, 148, 148, 148, 147, 147, 146, 146, 145, 145, 144, 143, 143, 142, 141, 140, 140, 139, 138, 137, 136, 135, 134, 133, 132, 130, 129, 128, 127, 126, 124, 123, 122, 121,
119, 118, 117, 115, 114, 113, 111, 110, 108, 107, 105, 104, 103, 101, 100,  98,  97,  95,  94,  92,  91,  89,  88,  87,  85,  84,  82,  81,  79,  78,  77,  75,  74,  72,  71,  70,  68,  67,  66,  64,
 63,  62,  60,  59,  58,  57,  55,  54,  53,  52,  51,  49,  48,  47,  46,  45,  44,  43,  42,  41,  40,  39,  38,  37,  36,  35,  34,  33,  32,  31,  31,  30,  29,  28,  27,  27,  26,  25,  24,  24,
 23,  22,  22,  21,  20,  20,  19,  18,  18,  17,  17,  16,  15,  15,  14,  14,  13,  13,  12,  12,  11,  11,  11,  10,  10,   9,   9,   9,   8,   8,   7,   7,   7,   6,   6,   6,   6,   5,   5,   5,
  4,   4,   4,   4,   3,   3,   3,   3,   3,   2,   2,   2,   2,   2,   2,   1,   1,   1,   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0
};


//************************************************
//* Breathing effect on Dot Matrix dislay
//*
//* brightness starts off for DELAY_BREATHING_OFF
//* then goes up to 0xFF then down back to 0
//************************************************
uint8_t Dotmatrix_Breathe(void)
{
    uint8_t change_word = 0;
    if (pwmVal == 0xFF)
    {
        F_DIR = 0;
        pwmVal--;
    }
    else if (pwmVal == 0)
    {
        if (breathing_zero_cnt == 0)
        {
            change_word = 1;
        }
        breathing_zero_cnt++;
        if (breathing_zero_cnt >= DELAY_BREATHING_OFF)
        {
            breathing_zero_cnt = 0;
            F_DIR = 1;
            pwmVal = 1;
        }
    }
    else if (F_DIR)
    {
        pwmVal++;
    }
    else
    {
        pwmVal--;
    }
    dotmatrix_set_brightness(pwmVal);
    return change_word;
}


void Dotmatrix_Breathe_Init(uint8_t pwm, uint8_t zero_cnt)
{
    breathing_word = 0;
    pwmVal = pwm;
    breathing_zero_cnt = zero_cnt;
    dotmatrix_set_brightness(pwm);
}


//=========================================================
void Do_Mode(void)
{
    if (F_INTERLOCK_CLOSE == 2 || F_NOZZLE_CLOSE == 2) return;

    switch(mode)
    {
        case MODE_SLEEP:
            Do_Sleep();
            break;

        case MODE_ADJUST_TIME:
            Do_Adjust_Time();
            break;

        case MODE_BLENDING:
            Do_Blending();
            break;

        case MODE_SPINNING:
            Do_Spinning();
            break;

        case MODE_ERROR:
            Do_Error();
            break;

#ifdef F_PCBA_TEST
        case MODE_PCBA_TEST:
            Do_Pcba_Test();
            break;
#endif

#ifdef F_FW_VERSION_DISPLAY
        case MODE_FW_VERSION:
            Do_Firmware_Version();
            break;
#endif
    }
    return;
}


void Do_Startup_Mes(void)
{
    if (mode != MODE_STARTUP_MES)
    {
        return;
    }

    mode_cnt++;

    if (mode_sec_cnt >= DELAY_STARTUP_MES)
    {
        if (breathing_zero_cnt >= DELAY_BREATHING_OFF)
        {
            if (F_AC_DETECT == AC_DETECT_NONE)
            {
                Go_To_Error(MODE_STEP_ERROR_AC_FREQ);
            }
            // fade-in time display if lid and nozzle are closed
            else if (F_INTERLOCK_CLOSE == 1 && F_NOZZLE_CLOSE == 1)
            {
                if (pwmVal == 0xFF)  // fade-in time display done
                {
                    Go_To_Adjust_Time();
                }
                else if (pwmVal > 0)  // fade-in time display
                {
                    pwmVal += 2;
                    if (pwmVal <= LED_BUTTON_PWM_MAX)
                    {
                        TIM17->CCR1 = pwmVal;
                    }
                    else
                    {
                        TIM17->CCR1 = LED_BUTTON_PWM_MAX;
                    }
                    dotmatrix_set_brightness(pwmVal);
                }
                else // fade-in time display start
                {
                    LED_DIAL_ARROW_ON();
                    pwmVal = 1;
                    TIM17->CCR1 = pwmVal;
                    dotmatrix_set_brightness(pwmVal);
                    showTimeOnly(adjust_time);
                }
            }
            else if (F_INTERLOCK_CLOSE == 1 || F_NOZZLE_CLOSE == 1)
            {
                Go_To_Adjust_Time();
            }
            else
            {
                Go_To_Sleep();
            }
        }
        else if (pwmVal >= 2)  // fade-out startup message
        {
            pwmVal -= 2;
            dotmatrix_set_brightness(pwmVal);
        }
        else if (pwmVal == 1)  // end of fade-out
        {
            pwmVal = 0;
            dotmatrix_set_brightness(pwmVal);
            clearDisplay();
        }
        else  // off time after fade-out
        {
            breathing_zero_cnt++;
        }
    }
    else if ((mode_cnt & 1) && pwmVal < 0xFF)  // fade-in startup message
    {
        pwmVal++;
        dotmatrix_set_brightness(pwmVal);
    }

    if (mode_cnt >= T_1SEC)
    {
        mode_cnt = 0;
        mode_sec_cnt++;
    }
}


//**********************************
//* Go To Sleep
//**********************************
void Go_To_Sleep(void)
{
    mode = MODE_SLEEP;
    mode_step = MODE_STEP_SLEEP_WAIT_DOTMATRIX_SRAM;
    LED_DIAL_ARROW_OFF();       // Turn off all LEDs
    dotmatrix_set_brightness(0);
    clearDisplay();             // Turn off LCD display
    F_MOTOR_RELAY = 0;          // Stop control output
    F_MOTOR_CCW = 0;
    F_MOTOR_CW = 0;
    Set_Triac_Off();
    F_CLICK = ENCODER_NO_CLICK;
    timed_out_sleep = 0;

    // enable Interlock, Encoder CCW, Button, Nozzle interrupts, used to cancel sleep if needed
    SET_BIT(EXTI->RPR1, 0xFFFF); // clear flags
    SET_BIT(EXTI->FPR1, 0xFFFF);
    SET_BIT(EXTI->RTSR1, EXTI_RTSR1_RT0 | EXTI_RTSR1_RT2 | EXTI_RTSR1_RT15); // Interlock, Encoder CCW, Nozzle
    SET_BIT(EXTI->FTSR1, EXTI_FTSR1_FT0 | EXTI_FTSR1_FT2 | EXTI_FTSR1_FT13 | EXTI_FTSR1_FT15); // Interlock, Encoder CCW, Button, Nozzle

    // init breathing DMA
    CLEAR_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);
    DMA1_Channel3->CMAR = (uint32_t)sleep_breathing_table; // source
    DMA1_Channel3->CNDTR = LED_BUTTON_SLEEP_STEPS;
    SET_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);

    // if led is off, start auto breathing right away
    if (TIM17->CCR1 == 0)
    {
        TIM17->DIER = TIM_DMA_UPDATE;
    }
    // if led is on, do fade-out transition manually in Do_Sleep()

    return;
}


/*************************************
 * Got to sleep because of timeout  **
 ************************************/
void Go_To_Sleep_Timeout(void)
{
    Go_To_Sleep();
    timed_out_sleep = 1;
}


//***********************************
//* Do Sleep
//***********************************
void Do_Sleep(void)
{
    // do fade-out transition if auto breathing is not on yet
    if ((TIM17->DIER & TIM_DMA_UPDATE) == 0)
    {
        if (TIM17->CCR1 > 0)
        {
            TIM17->CCR1 -= 1;
        }
        else
        {
            TIM17->DIER = TIM_DMA_UPDATE; // start auto breathing
        }
    }

    // check if lid/nozzle/encoder/button rising or falling edge happened
    if ( (EXTI->RPR1 & 0xFFFF) || (EXTI->FPR1 & 0xFFFF) )
    {
        SET_BIT(EXTI->RPR1, 0xFFFF); // clear flags
        SET_BIT(EXTI->FPR1, 0xFFFF);
        if (mode_step != MODE_STEP_SLEEP_WAKE_UP && mode == MODE_SLEEP)
        {
            timed_out_sleep = 0;    // only set for the first time we go to sleep
            F_NOZZLE_CLOSE = 2;     // reset nozzle and lid sensors
            F_INTERLOCK_CLOSE = 2;  // to check their actual state
            mode_step = MODE_STEP_SLEEP_WAKE_UP;
        }
    }

    // wait dotmatrix off before going to sleep
    else if (mode_step == MODE_STEP_SLEEP_WAIT_DOTMATRIX_SRAM)
    {
        if (SRAMbuffer0Complete == 0)
        {
            VSyncBufferSent = 0;
            mode_step = MODE_STEP_SLEEP_WAIT_DOTMATRIX_VSYNC;
        }
    }
    else if (mode_step == MODE_STEP_SLEEP_WAIT_DOTMATRIX_VSYNC)
    {
        // wait for VSYNC to be sent
        if ((VSyncBufferSent || TLC6983Enabled == 0) && DMA1_Channel2->CNDTR == 0 && (SPI2->SR & SPI_SR_FTLVL) == 0)
        {
            TLC6983Enabled = 0;
            mode_cnt = 0;
            mode_step = MODE_STEP_SLEEP_WAIT_LOW_POWER;
        }
    }
    else if (mode_step == MODE_STEP_SLEEP_WAIT_LOW_POWER)
    {
        if (mode_cnt <= T_2SEC)
        {
            mode_cnt++;
        }
        else if (TIM17->DIER & TIM_DMA_UPDATE)
        {
            // set Timer 14 to refresh watchdog every 20s
            TIM14->CR1 = 0; // stop timer
            TIM14->PSC = 999; // 1MHz / 1000 = 1kHz
            TIM14->EGR |= TIM_EVENTSOURCE_UPDATE; // to load the new prescaler
            TIM14->ARR = 19999; // period 20000ms = 20s
            TIM14->CNT = 19999 - DELAY_SLEEP_CHECK_PINS; // program next check 50ms after sleep to check for lid/nozzle pins
            TIM14->SR = 0; // Clear the pending interrupt flag
            TIM14->DIER = TIM_IT_UPDATE; // Enable the timer interrupt

            // disable peripheral power
            CLEAR_BIT(RCC->CFGR, RCC_CFGR_MCOPRE_Msk | RCC_CFGR_MCOSEL_Msk); // MCO used by dotmatrix
            bma400_sleep_mode();
            VOLTAGE_SUPPLY_OFF();

            // disable and clear interrupts not needed to get out of sleep
            CLEAR_BIT(DMA1_Channel1->CCR, DMA_IT_TC); // accelerometer DMA
            CLEAR_BIT(USART1->CR1, USART_CR1_RXNEIE_RXFNEIE);
            CLEAR_BIT(USART1->CR3, USART_CR3_EIE);
            HAL_SuspendTick(); // disable systick interrupt
            HAL_NVIC_DisableIRQ(USART1_IRQn);
            HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
            NVIC->ICPR[0] = 0xFFFFFFFFU; // clear all pending interrupts
            SET_BIT(SCB->ICSR, SCB_ICSR_PENDSVCLR_Msk | SCB_ICSR_PENDSTCLR_Msk); // clear PendSV and systick interrupt

            // switch to low power mode
            SET_BIT(RCC->CR, RCC_HSI_DIV16);            // lower system clock
            TIM17->PSC = 0;                             // set Timer 17 for lower clock
            TIM17->EGR |= TIM_EVENTSOURCE_UPDATE;       // to load the new prescaler
            SET_BIT(PWR->CR1, PWR_CR1_LPR);             // activate low power regulator
            while ((PWR->SR2 & PWR_SR2_REGLPF) == 0) ;  // wait for low power mode

            // slow down watchdog to 32.76s period
            IWDG->KR = IWDG_KEY_WRITE_ACCESS_ENABLE;
            SET_BIT(IWDG->PR, IWDG_PR_PR_Msk); // 32kHz/256

            // enable interrupts used to get out of sleep
            HAL_NVIC_EnableIRQ(INTERLOCK_EXTI_IRQn);
            HAL_NVIC_EnableIRQ(ENCODER_CCW_EXTI_IRQn);
            HAL_NVIC_EnableIRQ(ON_OFF_BUTTON_EXTI_IRQn); // Button and Nozzle
            TIM14->CR1 = TIM_CR1_CEN; // start timer 14 for watchdog
            // a timer wake up is scheduled in 50ms to check for lid/nozzle pin state

            // go to sleep mode
            SET_BIT(SCB->SCR, SCB_SCR_SLEEPONEXIT_Msk); // go back to sleep after each interrupt
            __WFI(); // stop CPU and wait for interrupt

            // restore watchdog freq
            CLR_WDT();
            IWDG->KR = IWDG_KEY_WRITE_ACCESS_ENABLE;
            CLEAR_BIT(IWDG->PR, IWDG_PR_PR_Msk);

            // disable lid/nozzle/button/encoder interrupts
            HAL_NVIC_DisableIRQ(INTERLOCK_EXTI_IRQn);
            HAL_NVIC_DisableIRQ(ENCODER_CCW_EXTI_IRQn);
            HAL_NVIC_DisableIRQ(ON_OFF_BUTTON_EXTI_IRQn); // Button and Nozzle

            // restore peripheral power
            TIM14->CR1 = 0; // stop timer
            TIM14->DIER = 0; // disable interrupt
            TIM14->PSC = 0; // 1MHz
            TIM14->EGR |= TIM_EVENTSOURCE_UPDATE; // to load the new prescaler
            TIM14->ARR = 20 - 1; // 20us
            TIM14->CNT = 0; // reset counter
            TIM14->SR = 0; // clear update flag
            TIM17->CR1 = 0;  // turn off button led timer to stop breathing DMA
            VOLTAGE_SUPPLY_ON();
            // wait 20us using timer 14
            TIM14->CR1 = TIM_CR1_CEN;  // start timer
            while ((TIM14->SR & TIM_SR_UIF) == 0) ;
            TIM17->CR1 = TIM_CR1_CEN; // turn on button led timer
            SET_BIT(RCC->CFGR, RCC_MCODIV_8 | RCC_MCO1SOURCE_SYSCLK); // MCO used by dotmatrix

            // exit low power mode
            CLEAR_BIT(PWR->CR1, PWR_CR1_LPR);
            while (PWR->SR2 & PWR_SR2_REGLPF) ; // wait for low power mode exit
            TIM17->PSC = 15; // set Timer 17 for high clock speed
            CLEAR_BIT(RCC->CR, RCC_HSI_DIV128); // restore system clock to full speed

            // use timer 14 to wait 1ms between accelerometer power and init
            // the prescaler is also restored to the correct value for TRIAC control
            TIM14->CR1 = 0; // stop timer
            TIM14->PSC = 15; // 16MHz / 16 = 1MHz
            TIM14->EGR |= TIM_EVENTSOURCE_UPDATE; // to load the new prescaler
            TIM14->ARR = 1000 - 1; // 1ms
            TIM14->CNT = 0; // reset counter
            TIM14->SR = 0; // clear update flag
            TIM14->CR1 = TIM_CR1_CEN; // start timer

            // re init dotmatrix and accelerometer
            InitialiseTLC6983();
            clearDisplay();
            // wait 1ms before init accelerometer
            while ((TIM14->SR & TIM_SR_UIF) == 0) ;
            TIM14->CR1 = 0; // stop timer
            bma400_init();

            // restore interrupts
            HAL_NVIC_EnableIRQ(USART1_IRQn);
            HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
            SET_BIT(USART1->CR1, USART_CR1_RXNEIE_RXFNEIE);
            SET_BIT(USART1->CR3, USART_CR3_EIE);
            SET_BIT(DMA1_Channel1->CCR, DMA_IT_TC); // accelerometer DMA
            HAL_ResumeTick(); // enable systick interrupt

            timed_out_sleep = 0;   // only set for the first time we go to sleep
            F_NOZZLE_CLOSE = 2;    // reset nozzle and lid sensors
            F_INTERLOCK_CLOSE = 2; // when we get out of sleep
            mode_step = MODE_STEP_SLEEP_WAKE_UP;
        }
    }

    // sleep exited
    if (mode_step == MODE_STEP_SLEEP_WAKE_UP)
    {
        // check lid/nozzle status to wake up or go back to sleep
        if (F_NOZZLE_CLOSE == 1 || F_INTERLOCK_CLOSE == 1)
        {
            // disable lid/nozzle/button/encoder interrupts
            CLEAR_BIT(EXTI->RTSR1, EXTI_RTSR1_RT0 | EXTI_RTSR1_RT2 | EXTI_RTSR1_RT15); // Interlock, Encoder CCW, Nozzle
            CLEAR_BIT(EXTI->FTSR1, EXTI_FTSR1_FT0 | EXTI_FTSR1_FT2 | EXTI_FTSR1_FT13 | EXTI_FTSR1_FT15); // Interlock, Encoder CCW, Button, Nozzle
            SET_BIT(EXTI->RPR1, 0xFFFF); // clear flags
            SET_BIT(EXTI->FPR1, 0xFFFF);
            TLC6983Enabled = 1;
            Go_To_Adjust_Time();
        }
#ifdef F_FW_VERSION_DISPLAY
        else if (F_NOZZLE_CLOSE == 0 && F_INTERLOCK_CLOSE == 0 && key_hold == 0)
        {
            mode_step = MODE_STEP_SLEEP_WAIT_DOTMATRIX_SRAM; // go back to sleep
        }
        else if (F_NOZZLE_CLOSE == 0 && F_INTERLOCK_CLOSE == 0 && key_hold && F_CLICK != ENCODER_NO_CLICK)
        {
            if (F_CLICK == ENCODER_CW_CLICK)
            {
                if (c_fw_version_clicks < T_CLICK_FW_VERSION)  // fw version mode: rotate clockwise first
                {
                    c_fw_version_clicks++;
                }
            }
            else if (F_CLICK == ENCODER_CCW_CLICK)
            {
                if (c_fw_version_clicks >= T_CLICK_FW_VERSION)  // fw version mode: then rotate counter clockwise
                {
                    c_fw_version_clicks++;
                    if (c_fw_version_clicks >= T_CLICK_FW_VERSION * 2 && key_hold >= T_KEY_HOLD)
                    {
                        // start fw version mode when both rotations are done and button is still pressed
                        mode = MODE_FW_VERSION;
                        mode_step = MODE_STEP_FW_VERSION_NUMBER;
                        dotmatrix_set_brightness(0xFF);
                        showString(VERSION_NUMBER, 1, 0);
                        TIM17->DIER = 0;  // disable breathing button led
                        LED_BUTTON_TEXT_OFF();
                        // disable lid/nozzle/button/encoder interrupts
                        CLEAR_BIT(EXTI->RTSR1, EXTI_RTSR1_RT0 | EXTI_RTSR1_RT2 | EXTI_RTSR1_RT15); // Interlock, Encoder CCW, Nozzle
                        CLEAR_BIT(EXTI->FTSR1, EXTI_FTSR1_FT0 | EXTI_FTSR1_FT2 | EXTI_FTSR1_FT13 | EXTI_FTSR1_FT15); // Interlock, Encoder CCW, Button, Nozzle
                        SET_BIT(EXTI->RPR1, 0xFFFF); // clear flags
                        SET_BIT(EXTI->FPR1, 0xFFFF);
                        TLC6983Enabled = 1;
                    }
                }
                else
                {
                    c_fw_version_clicks = 0; // reset counter if not turned in right order
                }
            }
            F_CLICK = ENCODER_NO_CLICK;
        }
#else   // F_FW_VERSION_DISPLAY
        else if (F_NOZZLE_CLOSE == 0 && F_INTERLOCK_CLOSE == 0)
        {
            mode_step = MODE_STEP_SLEEP_WAIT_DOTMATRIX_SRAM; // go back to sleep
        }
#endif  // F_FW_VERSION_DISPLAY
    }
    return;
}


//***********************************
//* Go To Adjust_Time
//***********************************
void Go_To_Adjust_Time(void)
{
    mode = MODE_ADJUST_TIME;
    F_MOTOR_RELAY = 0;
    F_MOTOR_CCW = 0;
    F_MOTOR_CW = 0;
    mode_step = MODE_STEP_ADJUST_TIME_CLOSE_MES;
    mode_cnt = 0;
    dotmatrix_update_time = 0;
    TIM17->DIER = 0;  // disable breathing button led
    if (F_INTERLOCK_CLOSE == 0 || (F_NOZZLE_CLOSE == 0 && adjust_time != 0))
    {
        // prepare for fade in of "close lid/spout" message
        Dotmatrix_Breathe_Init(0, DELAY_BREATHING_OFF);
        clearDisplay();
        LED_BUTTON_TEXT_OFF();
        LED_DIAL_ARROW_OFF(); // Turn off "Dial Arrow" LED
    }
    button_ready = 0;
    F_CLICK = ENCODER_NO_CLICK;
    timeout_mode = 0;                // Clear timeout counter
}


//*******************************
// Do Adjust_Time
//*******************************
void Do_Adjust_Time(void)
{
    if (F_INTERLOCK_CLOSE == 1 && (F_NOZZLE_CLOSE == 1 || adjust_time == 0))
    {
        // switch on display/led when nozzle/interlock are just closed
        if (mode_step == MODE_STEP_ADJUST_TIME_CLOSE_MES)
        {
            mode_step = MODE_STEP_ADJUST_TIME_READY;
            mode_cnt = 0;
            mode_sec_cnt = 0;
            button_ready = 1;
            LED_DIAL_ARROW_ON();
            LED_BUTTON_TEXT_ON();
            F_CLICK = ENCODER_NO_CLICK;
            dotmatrix_update_time = 0;
            if (adjust_time)
            {
                dotmatrix_set_brightness(0xFF);
                showTimeOnly(adjust_time);
            }
            else
            {
                Dotmatrix_Breathe_Init(0, DELAY_BREATHING_OFF);
                showSPIN();
            }
        }
        mode_cnt++;
        // set button ready after 100ms
        if (mode_cnt == T_01SEC)
        {
            button_ready = 1;
        }
        // count seconds to activate "set blend time"
        else if (mode_cnt >= T_1SEC)
        {
            mode_cnt = 0;
            if (adjust_time && mode_sec_cnt < DELAY_SET_BLEND_TIME)
            {
                mode_sec_cnt++;
                if (mode_sec_cnt == DELAY_SET_BLEND_TIME)
                {
                    Dotmatrix_Breathe_Init(0, DELAY_BREATHING_OFF);
                    showSET();
                }
            }
        }
        // "spin only" display
        if (adjust_time == 0)
        {
            if (Dotmatrix_Breathe())
            {
                breathing_word++;
                if (breathing_word == 1)
                {
                    showONLY();
                }
                else  // 0 or 2
                {
                    breathing_word = 0;
                    showSPIN();
                }
            }
        }
        // "set blend time" display
        else if (mode_sec_cnt >= DELAY_SET_BLEND_TIME)
        {
            if (Dotmatrix_Breathe())
            {
                breathing_word++;
                if (breathing_word == 2)
                {
                    showTIME();
                }
                else if (breathing_word == 1)
                {
                    showBLEND();
                }
                else  // 0 or 3
                {
                    breathing_word = 0;
                    showSET();
                }
            }
        }
        // encoder ajust time
        if (F_CLICK == ENCODER_CW_CLICK)
        {
            F_CLICK = ENCODER_NO_CLICK;
            button_ready = 0;
            mode_cnt = 0;
            if (adjust_time < L_ADJUST_MAX_TIME)
            {
                adjust_time += 5;
                dotmatrix_update_time = 1;
            }
            else if (mode_sec_cnt >= DELAY_SET_BLEND_TIME)
            {
                // only update display if "set blend time" is showing
                dotmatrix_update_time = 1;
            }
            mode_sec_cnt = 0;
        }
        else if (F_CLICK == ENCODER_CCW_CLICK)
        {
            F_CLICK = ENCODER_NO_CLICK;
            button_ready = 0;
            mode_cnt = 0;
            mode_sec_cnt = 0;
            if (adjust_time > L_ADJUST_MIN_TIME)  // update time on display
            {
                adjust_time -= 5;
                dotmatrix_update_time = 1;
            }
            else if (adjust_time)  // 0 reached, start fade-in "SPIN ONLY"
            {
                adjust_time = 0;
                timeout_mode = 0;
                dotmatrix_update_time = 0; // cancel previous display update
                Dotmatrix_Breathe_Init(0, DELAY_BREATHING_OFF);
                showSPIN();
            }
            else  // time is already 0, no display update, only reset timeout
            {
                timeout_mode = 0;
            }
        }
        // update time on dotmatrix only if there was no other update in the last 20ms to avoid glitches
        if (dotmatrix_update_time && timeout_mode >= DELAY_DOTMATRIX_TIME)
        {
            timeout_mode = 0;
            dotmatrix_update_time = 0;
            dotmatrix_set_brightness(0xFF);
            showTimeOnly(adjust_time);
        }
    }
    else if (F_NOZZLE_CLOSE == 0 && F_INTERLOCK_CLOSE == 0)  // lid and nozzle open / jug removed?
    {
        Go_To_Sleep();
    }
    else  // lid or nozzle open but not both at the same time
    {
        if (mode_step == MODE_STEP_ADJUST_TIME_READY)
        {
            button_ready = 0;  // disable button right away
            mode_step = MODE_STEP_ADJUST_TIME_CLOSE_MES; // show close lid/nozzle mode
            mode_cnt = 0;
            Dotmatrix_Breathe_Init(0, DELAY_BREATHING_OFF);
            clearDisplay();
        }

        // close lid/spout display
        if (mode_cnt >= T_01SEC)
        {
            if (Dotmatrix_Breathe())
            {
                breathing_word++;
                if (breathing_word == 1)
                {
                    if (F_INTERLOCK_CLOSE == 1)
                    {
                        showSPOUT();
                    }
                    else
                    {
                        showLID();
                    }
                }
                else // 0 or 2
                {
                    breathing_word = 0;
                    showCLOSE();
                }
            }
        }
        // delay first "close" message to prevent activating
        // when the jug is removed (will go to sleep mode before)
        else if (mode_cnt == (T_01SEC - 1))
        {
            mode_cnt++;
            showCLOSE();
            LED_BUTTON_TEXT_OFF();
            LED_DIAL_ARROW_OFF();
        }
        else
        {
            mode_cnt++;
        }
    }

    // decrease pour time if lid is closed and nozzle open
    if (pour_time && F_INTERLOCK_CLOSE == 1 && F_NOZZLE_CLOSE == 0)
    {
        pour_cnt++;
        if (pour_cnt >= T_1SEC)
        {
            pour_cnt = 0;
            pour_time--;
        }
    }

    Check_Timeout();
    return;
}


//*********************************
//* Go To Blending
//*********************************
void Go_To_Blending(void)
{
    mode = MODE_BLENDING; // Go to blending mode
    LED_DIAL_ARROW_OFF(); // Turn off "Dial Arrow" LED
    mode_step = MODE_STEP_BLEND_WAIT_LID;
    mode_cnt = 0;
    F_MOTOR_RELAY = 1; // Turn on Motor Relay
    F_MOTOR_CCW = 0; // Turn off CCW
    blend_time = adjust_time;
    CurrTargetSpeed = C_RPM_BLEND;
    pour_time = DEFAULT_POUR_TIME;
}


//*********************************
//* Do Blending
//*********************************
void Do_Blending(void)
{
    // if lid/nozzle/thermostat opens when motor is running jump to motor stop step
    if (MODE_STEP_BLEND_DIR_RELAY <= mode_step && mode_step < MODE_STEP_BLEND_MOTOR_STOP)
    {
        if (F_INTERLOCK_CLOSE == 0)  // Is interlock open
        {
            Set_Triac_Off();      // Turn off motor triac first
            mode_cnt = 0;
            exit_code = EXIT_INTERLOCK_OPEN;
            mode_step = MODE_STEP_BLEND_MOTOR_STOP;
        }
        else if (F_THERMOSTAT_CLOSE == 0 && F_MOTOR_RELAY == 1 && F_MOTOR_TRIAC == 1)    // Is thermostat open?
        {
            Set_Triac_Off();      // Turn off motor triac first
            mode_cnt = 0;
            exit_code = EXIT_THERMOSTAT_OPEN;
            mode_step = MODE_STEP_BLEND_MOTOR_STOP;
        }
        else if (F_NOZZLE_CLOSE == 0)        // Is nozzle open?
        {
            Set_Triac_Off();      // Turn off motor triac first
            mode_cnt = 0;
            exit_code = EXIT_NOZZLE_OPEN;
            mode_step = MODE_STEP_BLEND_MOTOR_STOP;
        }
    }
    else if (mode_step < MODE_STEP_BLEND_DIR_RELAY && F_INTERLOCK_CLOSE == 0 && F_NOZZLE_CLOSE == 0)  // jug possibly removed
    {
        Go_To_Sleep();
        return;
    }

    switch(mode_step)
    {
        case MODE_STEP_BLEND_WAIT_LID:  // wait lid and display "close lid"
            if (F_INTERLOCK_CLOSE == 1)
            {
                dotmatrix_set_brightness(0xFF);
                showTimeOnly(blend_time);
                LED_BUTTON_TEXT_ON();
                mode_sec_cnt =  0;
                mode_step++;
            }
            else  // show "close lid"
            {
                if (Dotmatrix_Breathe())
                {
                    breathing_word++;
                    if (breathing_word == 1)
                    {

                        showLID();
                    }
                    else // 0 or 2
                    {
                        breathing_word = 0;
                        showCLOSE();
                    }
                }

                Check_Timeout();
            }
            break;

        case MODE_STEP_BLEND_DELAY_RESUME:  // delay resume blending
            mode_cnt++;
            if (F_MOTOR_RELAY == 1)  // skip step if motor relay is already on (blending started by button)
            {
                mode_step++;
                mode_cnt++; // increase mode_cnt to account for skipped step 0
            }
            else if (F_INTERLOCK_CLOSE == 1)
            {
                if (F_NOZZLE_CLOSE == 0)  // nozzle open, go to spinning
                {
                    F_MOTOR_RELAY = 0;
                    Go_To_Spinning();
                }
                else if (mode_cnt >= T_1SEC)  // wait delay before resuming motor
                {
                    mode_cnt = 0;
                    mode_sec_cnt++;
                    if (mode_sec_cnt >= DELAY_BLEND_RESUME)
                    {
                        F_MOTOR_RELAY = 1; // Turn on Motor Relay
                        mode_step++;
                    }
                }
            }
            else  // lid is re opened, go back to wait lid step
            {
                Dotmatrix_Breathe_Init(0, DELAY_BREATHING_OFF);
                showCLOSE();
                LED_BUTTON_TEXT_OFF();
                mode_cnt = 0;
                mode_step--;
            }
            break;

        case MODE_STEP_BLEND_DIR_RELAY:  // turn on direction relay
            mode_cnt++;
            if (mode_cnt >= T_52MS)
            {
                F_MOTOR_CW = 1; // Then turn on Motor CW relay
                mode_step++;
            }
            break;

        case MODE_STEP_BLEND_PID_INIT:  // init PID and turn on Triac
            mode_cnt++;
            if (mode_cnt >= T_01SEC)
            {
                mode_cnt = 0;
                MotorPIDInit(PID_Kp_CW, PID_Ki_CW, PID_Kd_CW, C_PID_Power_INIT_BLEND, C_SPEED_PERIOD_BLEND);
                mode_step++;
            }
            break;

        case MODE_STEP_BLEND_MOTOR_RUN:  // motor run step
        {
            if (PID_Time >= C_Inc_PID_Sample_Time)
            {
                PID_Time = 0;
                MotorPIDControl(CurrTargetSpeed);
                mode_cnt++; // increment mode count with PID_Time so display update is run just after PID
                // speed check
                if (Check_Motor_Speed(DELAY_CHK_SPEED_BLEND, C_RPM_BLEND_LOW, C_RPM_BLEND_HIGH, C_RPM_BLEND_MAX))
                {
                    Set_Triac_Off();
                    mode_cnt = 0;
                    exit_code = EXIT_MOTOR_SPEED_ERROR;
                    mode_step++;
                }
                else if (mode_cnt >= T_1SEC_PID)
                {
                    mode_cnt = 0;
                    // decrement blend time
                    blend_time--;
                    showTimeOnly(blend_time);
                    if (blend_time == 0)
                    {
                        Set_Triac_Off();
                        exit_code = EXIT_OPERATION_COMPLETE;
                        mode_step++;
                    }
                    // check AC signal
                    else if (hz_count == 0)
                    {
                        Set_Triac_Off();
                        mode_cnt = 0;
                        exit_code = EXIT_AC_SIGNAL_ERROR;
                        mode_step++;
                    }
                    hz_count = 0;
                }
            }
            break;
        }

        case MODE_STEP_BLEND_MOTOR_STOP:  // turn off motor relay
            mode_cnt++;
            if (mode_cnt >= T_52MS)
            {
                mode_cnt = 0;
                F_MOTOR_RELAY = 0;
                mode_step++;
            }
            break;

        case MODE_STEP_BLEND_EXIT_CODE:  // process exit_code
            mode_cnt++;
            if (mode_cnt >= T_52MS)
            {
                mode_cnt = 0;
                F_MOTOR_CW = 0;  // Turn off CW Relay
                switch (exit_code)
                {
                case EXIT_ONOFF_PRESS:
                    Go_To_Adjust_Time();
                    break;
                case EXIT_INTERLOCK_OPEN:  // pause when lid is opened during blend
                    mode_step = MODE_STEP_BLEND_WAIT_LID; // go back to wait lid step
                    Dotmatrix_Breathe_Init(0, DELAY_BREATHING_OFF);
                    showCLOSE();
                    LED_BUTTON_TEXT_OFF();
                    break;
                case EXIT_NOZZLE_OPEN:
                    if (F_INTERLOCK_CLOSE == 1)
                    {
                        showTimeOnly(DEFAULT_POUR_TIME);
                    }
                    LED_BUTTON_TEXT_OFF();
                    mode_step++;
                    break;
                case EXIT_THERMOSTAT_OPEN:
                    Go_To_Error(MODE_STEP_ERROR_THERMOSTAT);
                    break;
                case EXIT_OPERATION_COMPLETE:
                    mode_sec_cnt = 0;
                    mode_step++;
                    break;
                case EXIT_MOTOR_SPEED_ERROR:
                    Go_To_Error(MODE_STEP_ERROR_BLEND);
                    break;
                case EXIT_AC_SIGNAL_ERROR:
                    Go_To_Error(MODE_STEP_ERROR_AC_FREQ);
                    break;
                }
            }
            break;

        case MODE_STEP_BLEND_END:  // delay before going to spin mode
            mode_cnt++;
            if (F_INTERLOCK_CLOSE == 0 && F_NOZZLE_CLOSE == 0)
            {
                Go_To_Sleep();
            }
            else if (mode_cnt >= T_1SEC)
            {
                mode_cnt = 0;
                mode_sec_cnt++;
                if (exit_code == EXIT_ONOFF_PRESS)
                {
                    Go_To_Adjust_Time();
                }
                else if (mode_sec_cnt >= DELAY_BLEND_DONE || exit_code == EXIT_NOZZLE_OPEN)
                {
                    Go_To_Spinning();
                }
            }
            break;
    }
    return;
}


//*************************
//* Go To Spinning
//*************************
void Go_To_Spinning(void)
{
    // check if accelerometer is detected
#ifndef F_VIBRATION_LIMIT_DISABLE
    if (bma400_get_sensor_detected() == 0)
    {
        Go_To_Error(MODE_STEP_ERROR_ACCEL);
        return;
    }
#endif

    mode = MODE_SPINNING;
    spin_time = DEFAULT_SPIN_TIME;
    mode_cnt = 0;
    timeout_mode = 0;
    if (F_INTERLOCK_CLOSE == 0 && F_NOZZLE_CLOSE == 1)
    {
        Dotmatrix_Breathe_Init(0, DELAY_BREATHING_OFF);
        showCLOSE();
        mode_step = MODE_STEP_SPIN_WAIT_LID; // wait lid close step
    }
    else if (F_NOZZLE_CLOSE == 1)
    {
        Dotmatrix_Breathe_Init(0, DELAY_BREATHING_OFF);
        showOPEN();
        mode_step = MODE_STEP_SPIN_WAIT_NOZZLE; // wait nozzle open step
    }
    else if (pour_time)
    {
        if (pour_time < DEFAULT_POUR_TIME_MIN)  // prevent having a pour time step too short
        {
            pour_time = DEFAULT_POUR_TIME_MIN;
        }
        dotmatrix_set_brightness(0xFF);
        showTimeOnly(pour_time);
        mode_step = MODE_STEP_SPIN_POURING; // pouring step
    }
    else // pour time is zero
    {
        F_MOTOR_RELAY = 1; // Turn on Motor Relay first
        dotmatrix_set_brightness(0xFF);
        showTimeOnly(spin_time);
        LED_BUTTON_TEXT_ON();
        mode_step = MODE_STEP_SPIN_DIR_RELAY; // start spinning
    }
    LED_BUTTON_TEXT_OFF();
    LED_DIAL_ARROW_OFF(); // Turn off "Dial Arrow" LED
    F_MOTOR_CW = 0;
}


//*********************************
//* Do Spinning
//*********************************
void Do_Spinning(void)
{
    // if lid/nozzle/thermostat opens when motor is running jump to motor stop step
    if (MODE_STEP_SPIN_DIR_RELAY <= mode_step && mode_step < MODE_STEP_SPIN_MOTOR_STOP)
    {
        if (F_THERMOSTAT_CLOSE == 0 && F_MOTOR_RELAY == 1 && F_MOTOR_TRIAC == 1)    // Is thermostat open?
        {
            Set_Triac_Off();              // Turn off Motor Triac first
            mode_cnt = 0;
            exit_code = EXIT_THERMOSTAT_OPEN;
            mode_step = MODE_STEP_SPIN_MOTOR_STOP; //Stop spinning
        }
        else if (F_NOZZLE_CLOSE == 1)
        {
            Set_Triac_Off();              // Turn off Motor Triac first
            mode_cnt = 0;
            exit_code = EXIT_NOZZLE_CLOSE;
            mode_step = MODE_STEP_SPIN_MOTOR_STOP; //Stop spinning
        }
        // lid can be opened in pouring step when waiting for the nozzle to open
        else if (F_INTERLOCK_CLOSE == 0)  // Is interlock open
        {
            Set_Triac_Off(); // Turn off motor triac first
            mode_cnt = 0;
            exit_code = EXIT_INTERLOCK_OPEN;
            mode_step = MODE_STEP_SPIN_MOTOR_STOP;
        }
    }
    else if (mode_step < MODE_STEP_SPIN_DIR_RELAY && F_INTERLOCK_CLOSE == 0 && F_NOZZLE_CLOSE == 0) // jug possibly removed
    {
        Go_To_Sleep();
        return;
    }

    switch(mode_step)
    {
        case MODE_STEP_SPIN_WAIT_LID:  // wait lid close
            if (F_INTERLOCK_CLOSE == 1)
            {
                pour_time = DEFAULT_POUR_TIME; // reset pour time because liquid might have beeen added
                if (F_NOZZLE_CLOSE == 1)
                {
                    Dotmatrix_Breathe_Init(0, DELAY_BREATHING_OFF);
                    showOPEN();
                    mode_step++;
                }
                else
                {
                    dotmatrix_set_brightness(0xFF);
                    showTimeOnly(pour_time);
                    mode_step = MODE_STEP_SPIN_POURING; // go to pouring, skip wait nozzle
                }
            }
            else // show close lid
            {
                if (Dotmatrix_Breathe())
                {
                    breathing_word++;
                    if (breathing_word == 1)
                    {
                        showLID();
                    }
                    else // 0 or 2
                    {
                        breathing_word = 0;
                        showCLOSE();
                    }
                }
                Check_Timeout();
            }
            break;

        case MODE_STEP_SPIN_WAIT_NOZZLE:  // wait nozzle open
            if (F_NOZZLE_CLOSE == 0 && F_INTERLOCK_CLOSE == 1)
            {
                dotmatrix_set_brightness(0xFF);
                if (pour_time)
                {
                    showTimeOnly(pour_time);
                    mode_step++;
                }
                else  // skip pouring
                {
                    showTimeOnly(spin_time);
                    LED_BUTTON_TEXT_ON();
                    F_MOTOR_RELAY = 1;
                    mode_step = MODE_STEP_SPIN_DIR_RELAY;
                }
            }
            else if (F_INTERLOCK_CLOSE == 0)
            {
                Dotmatrix_Breathe_Init(0, DELAY_BREATHING_OFF);
                showCLOSE();
                mode_step--;
            }
            else  // show open spout
            {
                if (Dotmatrix_Breathe())
                {
                    breathing_word++;
                    if (breathing_word == 1)
                    {
                        showSPOUT();
                    }
                    else // 0 or 2
                    {
                        breathing_word = 0;
                        showOPEN();
                    }
                }
                Check_Timeout();
            }
            break;

        case MODE_STEP_SPIN_POURING:  // pouring
            mode_cnt++;
            if (F_INTERLOCK_CLOSE == 0)
            {
                Dotmatrix_Breathe_Init(0, DELAY_BREATHING_OFF);
                showCLOSE();
                mode_step = MODE_STEP_SPIN_WAIT_LID; // go to wait lid close
            }
            else if (F_NOZZLE_CLOSE == 1)
            {
                Dotmatrix_Breathe_Init(0, DELAY_BREATHING_OFF);
                showOPEN();
                mode_step = MODE_STEP_SPIN_WAIT_NOZZLE; // got to wait nozzle open
            }
            else if (mode_cnt >= T_1SEC)
            {
                mode_cnt = 0;
                if (pour_time > 1)  // some pour time left
                {
                    pour_time--;
                    showTimeOnly(pour_time);
                }
                else
                {
                    pour_time = 0;
                    F_MOTOR_RELAY = 1; // Turn on Motor Relay first
                    showTimeOnly(spin_time);
                    LED_BUTTON_TEXT_ON();
                    mode_step++;
                }
            }
            break;

        case MODE_STEP_SPIN_DIR_RELAY:  // turn on direction relay
            mode_cnt++;
            if (mode_cnt >= T_52MS)
            {
                F_MOTOR_CCW = 1;
                mode_step++;
            }
            break;

        case MODE_STEP_SPIN_PID_INIT:  // init PID and turn on Triac
            mode_cnt++;
            if (mode_cnt >= T_52MS)
            {
                mode_cnt = 0;
                SpinSpeed = C_RPM_SPIN_MIN;
                CurrTargetSpeed = SpinSpeed;
                MotorPIDInit(PID_Kp_CCW, PID_Ki_CCW, PID_Kd_CCW, C_PID_Power_INIT_SPIN, C_SPEED_PERIOD_SPIN);
#ifndef F_VIBRATION_LIMIT_DISABLE
                vibration_controller_init(C_RPM_SPIN_MIN + C_RPM_SPIN_INC*T_1SEC_PID);
#endif
                mode_step++;
            }
            break;

        case MODE_STEP_SPIN_MOTOR_RUN:  // run motor
            if (vibration_read_step == 0 && PID_Time >= (C_Inc_PID_Sample_Time - 2))
            {
                vibration_read_step++;
#ifndef F_VIBRATION_LIMIT_DISABLE
                bma400_read_fifo_start();
#endif
            }
            if (vibration_read_step == 1 && PID_Time >= (C_Inc_PID_Sample_Time - 1))
            {
                vibration_read_step++;
                // ramp up speed every 100 ms
                if (SpinSpeed < C_RPM_SPIN)
                {
                    SpinSpeed += C_RPM_SPIN_INC;
                    if (SpinSpeed > C_RPM_SPIN)
                    {
                        SpinSpeed = C_RPM_SPIN;
                    }
                }
#ifndef F_VIBRATION_LIMIT_DISABLE
                if (vibration_init >= T_1SEC_PID)  // start controlling vibration after 1 sec
                {
                    CurrTargetSpeed = vibration_controller(SpinSpeed);
                }
                else
                {
                    vibration_init++;
                    bma400_read_fifo(); // read fifo to init vibration calculation
                    vibration = 0;
                    CurrTargetSpeed = SpinSpeed;
                }
#else
                CurrTargetSpeed = SpinSpeed;
#endif
            }
            if (PID_Time >= C_Inc_PID_Sample_Time)
            {
                PID_Time = 0;
                MotorPIDControl(CurrTargetSpeed);
                vibration_read_step = 0;
                mode_cnt++; // increment mode count with PID_Time so display update is run just after PID
                // speed check
                uint16_t speed_low = C_RPM_SPIN_LOW;
                if (CurrTargetSpeed < C_RPM_SPIN)
                {
                    speed_low = C_RPM_SPIN_MIN;
                }
                if (Check_Motor_Speed(DELAY_CHK_SPEED_SPIN, speed_low, C_RPM_SPIN_HIGH, C_RPM_SPIN_MAX))
                {
                    Set_Triac_Off();
                    mode_cnt = 0;
                    exit_code = EXIT_MOTOR_SPEED_ERROR;
                    mode_step++;
                }
                else if (mode_cnt >= T_1SEC_PID)
                {
                    mode_cnt = 0;
                    // decrement spin time
                    spin_time--;
                    showTimeOnly(spin_time);
                    if (spin_time == 0)
                    {
                        Set_Triac_Off(); // Turn off Motor Triac first
                        exit_code = EXIT_OPERATION_COMPLETE;
                        mode_step++; //Stop spinning
                    }
                    // check AC signal
                    else if (hz_count == 0)
                    {
                        Set_Triac_Off();
                        exit_code = EXIT_AC_SIGNAL_ERROR;
                        mode_step++;
                    }
                    hz_count = 0;
                }
            }
            break;

        case MODE_STEP_SPIN_MOTOR_STOP:  // turn off motor relay
            mode_cnt++;
            if (mode_cnt >= T_52MS)
            {
                mode_cnt = 0;
                F_MOTOR_RELAY = 0;
                mode_step++;
            }
            break;

        case MODE_STEP_SPIN_EXIT_CODE:  // process exit_code
            mode_cnt++;
            if (mode_cnt >= T_52MS)
            {
                mode_cnt = 0;
                F_MOTOR_CCW = 0;

                switch (exit_code)
                {
                case EXIT_ONOFF_PRESS:
                    Go_To_Adjust_Time();
                    break;
                case EXIT_INTERLOCK_OPEN:
                case EXIT_NOZZLE_CLOSE:
                    if (exit_code == EXIT_INTERLOCK_OPEN)
                    {
                        Go_To_Adjust_Time();
                    }
                    else if (F_INTERLOCK_CLOSE == 0)
                    {
                        Dotmatrix_Breathe_Init(0, DELAY_BREATHING_OFF);
                        showCLOSE();
                        LED_BUTTON_TEXT_OFF();
                        mode_step = MODE_STEP_SPIN_WAIT_LID; // go to wait lid close step
                    }
                    else
                    {
                        Dotmatrix_Breathe_Init(0, DELAY_BREATHING_OFF);
                        showOPEN();
                        LED_BUTTON_TEXT_OFF();
                        mode_step = MODE_STEP_SPIN_WAIT_NOZZLE;   // got to wait open nozzle step
                    }
                    break;
                case EXIT_THERMOSTAT_OPEN:
                    Go_To_Error(MODE_STEP_ERROR_THERMOSTAT);
                    break;
                case EXIT_OPERATION_COMPLETE:
                    pwmVal = 0;
                    dotmatrix_set_brightness(0);
                    showENJOY();
                    LED_BUTTON_TEXT_OFF();
                    timeout_mode = 1; // set timeout to non zero, when it's changed to 0 it means lid/nozzle sensor changed state
                    mode_step++;
                    break;
                case EXIT_MOTOR_SPEED_ERROR:
                    Go_To_Error(MODE_STEP_ERROR_SPIN);
                    break;
                case EXIT_AC_SIGNAL_ERROR:
                    Go_To_Error(MODE_STEP_ERROR_AC_FREQ);
                    break;
                }
            }
            break;

        case MODE_STEP_SPIN_END:  // show "ENJOY"
            if (F_INTERLOCK_CLOSE == 0 && F_NOZZLE_CLOSE == 0)
            {
                Go_To_Sleep();
            }
            else if (pwmVal == 0xFF)  // exit "enjoy" mode after fade-in is done
            {
                if (F_CLICK == ENCODER_CW_CLICK || F_CLICK == ENCODER_CCW_CLICK)
                {
                    F_CLICK = ENCODER_NO_CLICK;
                    Go_To_Adjust_Time();
                }
                else if (timeout_mode == 0 || exit_code == EXIT_ONOFF_PRESS)  // change in lid/nozzle or button
                {
                    Go_To_Adjust_Time();
                }
                else
                {
                    Check_Timeout();
                }
            }
            else  // fade in "enjoy"
            {
                mode_cnt++;
                if (mode_cnt & 1)
                {
                    pwmVal++;
                    dotmatrix_set_brightness(pwmVal);
                }
            }
            break;
    }
}


//******************************
//* Go To Error
//******************************
void Go_To_Error(uint8_t error_mode)
{
    mode = MODE_ERROR;
    mode_step = error_mode;
    dotmatrix_set_brightness(0xFF);
    showERROR();
    LED_BUTTON_TEXT_OFF();
    LED_DIAL_ARROW_OFF();
    mode_cnt = 0;
}


//**************************************
//* Do Error
//**************************************
void Do_Error(void)
{
    mode_cnt++;
    if (mode_cnt == T_1_05SEC)
    {
        switch (mode_step)
        {
        case MODE_STEP_ERROR_THERMOSTAT:
            showTHERM();
            break;
        case MODE_STEP_ERROR_SPIN:
            showSPIN();
            break;
        case MODE_STEP_ERROR_BLEND:
            showBLEND();
            break;
        case MODE_STEP_ERROR_AC_FREQ:
            showACHz();
            break;
        case MODE_STEP_ERROR_ACCEL:
            showACCEL();
            break;
        }
    }
    else if (mode_cnt >= T_3SEC)
    {
        mode_cnt = 0;
        showERROR();
    }
    return;
}


#ifdef F_FW_VERSION_DISPLAY
//**************************************
//* Do Firmware version
//* Rotate through: fw version, commit offset,
//* commit hash, commit date, build config
//**************************************
void Do_Firmware_Version(void)
{
    if (F_CLICK != 0)
    {
        if (F_CLICK == ENCODER_CW_CLICK)
        {
            mode_step++;
            switch (mode_step)
            {
            default:
            case MODE_STEP_FW_VERSION_NUMBER:
                mode_step = MODE_STEP_FW_VERSION_NUMBER;
                showString(VERSION_NUMBER, 1, 0);
                break;
            case MODE_STEP_FW_VERSION_OFFSET:
                showString(VERSION_OFFSET, 1, 0);
                break;
            case MODE_STEP_FW_VERSION_COMMIT:
                showString(VERSION_COMMIT, 1, 0);
                break;
            case MODE_STEP_FW_VERSION_DATE:
                showString(VERSION_DATE, 1, 0);
                break;
            case MODE_STEP_FW_VERSION_BUILD:
                showString(VERSION_BUILD, 1, 0);
                break;
            }
        }
        F_CLICK = ENCODER_NO_CLICK;
    }
}
#endif


void Check_Timeout(void)
{
    if (timeout_mode < (T_5MIN - 1))
    {
        timeout_mode++;
    }
    else
    {
        Go_To_Sleep_Timeout();
    }
}
