#include "constant.h"

#ifdef F_PCBA_TEST
#include "stm32g0xx_hal.h"
#include "memory.h"
#include "TLC6983.h"
#include "BMA400.h"

extern void DisplayFullOn(void);
extern void showFAIL(void);
extern void showPASS(void);
extern void showHz(uint8_t ac_detect);
extern void showAccelCheck(uint8_t pass);
extern void showDirCheck(uint8_t direction, uint8_t pass);
extern void showInterlockNozzleCheck(uint8_t InterlockNozzle, uint8_t check);
extern void showString(char *thisString, uint8_t autoCenter, uint8_t fixedFontWidth);
extern void Go_To_Sleep(void);
extern void Go_To_Sleep_Timeout(void);
extern void MotorPIDControl(uint16_t TargetSpeed);
extern void MotorPIDInit(int32_t kp, int32_t ki, int32_t kd, uint16_t output_init, uint8_t speed_period);
void Motor_Thermostat_Init(void);
extern void Set_Triac_Off(void);

extern IWDG_HandleTypeDef hiwdg;

void Check_Test(void)
{
    if (mode == MODE_PCBA_TEST || test_substep == 1)
    {
        return;
    }

    if (ONOFF_KEY_PIN_STATUS() == 0)
    {
        test_substep = 1;  // disable test mode
        key_hold = 0;
    }

    if (key_hold >= T_KEY_HOLD)
    {
        mode = MODE_PCBA_TEST;
        mode_cnt = 0;
        mode_sec_cnt = 0;
        mode_step = 0;
        button_ready = 1;
        LED_DIAL_ARROW_ON(); // Turn on all LEDs
        LED_BUTTON_TEXT_ON();
        dotmatrix_set_brightness(0xFF);
        DisplayFullOn(); //Turn on all LED Display (36 x 8) on 3s for dead pixels checking
    }
    return;
}

void Do_Pcba_Test(void)
{
    mode_cnt++;
    if (mode_cnt >= T_1SEC)
    {
        mode_cnt = 0;
        mode_sec_cnt++;
        if (mode_sec_cnt >= 30)
        {
            Go_To_Sleep_Timeout();
            return;
        }
    }

    switch (mode_step)
    {
    case 0: // test all leds and dot matrix
        // just wait for button press
        break;

    case 1: // test only used led on dot matrix
        if (test_substep == 0)
        {
            fillOnlyUsedLeds();
            button_ready = 1;
            test_substep = 1;
        }
        break;

    case 2: // show version
        if (test_substep == 0)
        {
            showString(VERSION_NUMBER, 1, 6);
            button_ready = 1;
            test_substep = 1;
        }
        break;

    case 3: // test AC frequency detection
        if (test_substep == 0)
        {
            if (F_AC_DETECT != AC_DETECT_NONE)
            {
                showHz(F_AC_DETECT);
                button_ready = 1;
            }
            else // AC detection timed out (at least 2s spent to get to this step)
            {
                showString("AC ERR", 1, 6);
            }
            test_substep = 1;
        }
        break;

    case 4: // test interlock micro-switch
        switch (test_substep)
        {
        case 0:
        case 2:
            LED_BUTTON_TEXT_ON();
            if (F_INTERLOCK_CLOSE == 0) // INTERLOCK is open
            {
                mode_cnt = 0;
                mode_sec_cnt = 0;
                showInterlockNozzleCheck('I', test_substep);
                test_substep++;
            }
            break;
        case 1:
            if (F_INTERLOCK_CLOSE == 1) // INTERLOCK is close
            {
                mode_sec_cnt = 0;
                showInterlockNozzleCheck('I', test_substep);
                test_substep++;
            }
            break;
        case 3:
            if (mode_sec_cnt >= 1)
            {
                mode_sec_cnt = 0;
                test_substep = 0;
                mode_step++;
            }
            break;
        }
        if (test_substep < 4 && mode_sec_cnt >= 10)
        {
            showFAIL();
            mode_sec_cnt = 0;
            test_substep = 4;
        }
        break;

    case 5: // test nozzle switch
        switch (test_substep)
        {
        case 0:
            if (F_NOZZLE_CLOSE == 0) // Nozzle is open
            {
                mode_sec_cnt = 0;
                showInterlockNozzleCheck('N', test_substep);
                test_substep++;
            }
            break;
        case 1:
            if (F_NOZZLE_CLOSE == 1) // Nozzle is close
            {
                mode_sec_cnt = 0;
                showInterlockNozzleCheck('N', test_substep);
                test_substep++;
            }
            break;
        case 2:
            if (F_NOZZLE_CLOSE == 0) // Nozzle is open
            {
                mode_sec_cnt = 0;
                showInterlockNozzleCheck('N', test_substep);
                button_ready = 1;
                test_substep++;
            }
            break;
        }
        if (test_substep < 3 && mode_sec_cnt >= 10)
        {
            mode_sec_cnt = 0;
            showFAIL();
            test_substep = 3;
        }
        break;

    case 6: // test accelerometer (check if sensor detected and check if vibration value is low enough)
        if (test_substep == 0)
        {
            if (bma400_get_sensor_detected() == 1)
            {
                LED_BUTTON_TEXT_ON();
                for (uint8_t i = 0; i < 12; i++)  // read enough to fill the circular buffer
                {
                    bma400_read_fifo_start();
                    while (bma400_get_dma_count())  // wait DMA transfer
                    {
                        CLR_WDT();
                    }
                    bma400_read_fifo();
                }
                if (check_accel_data())
                {
                    showAccelCheck('P');
                    button_ready = 1;
                }
                else
                {
                    showAccelCheck('F');
                }
            }
            else
            {
                showAccelCheck('F');
            }
            test_substep++;
        }
        break;

    case 7: // test encoder CW
        if (test_substep == 0)
        {
            LED_DIAL_ARROW_ON();
            F_CLICK = ENCODER_NO_CLICK;
            test_substep++;
        }
        else if (test_substep == 1)
        {
            if (F_CLICK == ENCODER_CW_CLICK)
            {
                mode_sec_cnt = 0;
                F_CLICK = ENCODER_NO_CLICK;
                showDirCheck(C_DIRECTION_CW, 'P');
                test_substep = 0;
                mode_step++;
            }
            else if (mode_sec_cnt >= 10)
            {
                mode_sec_cnt = 0;
                showDirCheck(C_DIRECTION_CW, 'F');
                test_substep++;
            }
        }
        break;

    case 8: // test encoder CCW
        if (test_substep == 0)
        {
            if (F_CLICK == ENCODER_CCW_CLICK)
            {
                F_CLICK = ENCODER_NO_CLICK;
                showDirCheck(C_DIRECTION_CCW, 'P');
                mode_sec_cnt = 0;
                test_substep++;
                button_ready = 1;
            }
            else if (mode_sec_cnt >= 10)
            {
                mode_sec_cnt = 0;
                showDirCheck(C_DIRECTION_CCW, 'F');
                test_substep++;
            }
        }
        break;

    case 9: // test thermostat
        switch(test_substep)
        {
        case 0:
            F_MOTOR_RELAY = 1;
            Motor_Thermostat_Init();
            F_THERMOSTAT_CLOSE = 0;  // set to open first to test closed detection
            test_substep++;
            break;
        case 1:
            if (F_THERMOSTAT_CLOSE == 1 && mode_sec_cnt >= 1) //thermostat is close
            {
                mode_sec_cnt = 0;
                showPASS();
                test_substep++;
            }
            break;
        case 2:
            if (F_THERMOSTAT_CLOSE == 0 && mode_sec_cnt >= 1) // thermostat is open
            {
                mode_sec_cnt = 0;
                showPASS();
                LED_BUTTON_TEXT_ON();
                button_ready = 1;
                test_substep++;
            }
            break;
        }
        if (test_substep < 3 && mode_sec_cnt >= 10)
        {
            F_MOTOR_RELAY = 0;
            Set_Triac_Off();  // to disable thermostat interrupt
            showFAIL();
            test_substep = 3;
            mode_sec_cnt = 0;
        }
        break;

    case 10: // test motor CW (Blend)
        // After 0.05sec, turn CW relay on
        if (mode_cnt > T_52MS)
        {
            F_MOTOR_CW = 1;         // Then turn on Motor CW relay
            mode_step++;
        }
        break;
    case 11:
        // After 0.05sec, turn motor triac on and Display CW
        if (mode_cnt > T_01SEC)
        {
            showDirCheck(C_DIRECTION_CW, 0);
            CurrTargetSpeed = C_MOTOR_CW_TARGET;
            MotorPIDInit(PID_Kp_CW, PID_Ki_CW, PID_Kd_CW, C_PID_Power_INIT_BLEND, C_SPEED_PERIOD_BLEND);
            mode_step++;
        }
        break;
    case 12:
        // Run motor for 2.9sec then turn off motor triac
        if (PID_Time >= C_Inc_PID_Sample_Time)
        {
            PID_Time = 0;
            MotorPIDControl(CurrTargetSpeed);
        }
        // Pass if motor Speed has reached the minimum
        if (test_substep == 0 && CurrentSpeed >= C_MOTOR_CW_MIN && mode_sec_cnt >= 1)    // >19000rpm
        {
            test_substep = 1;
        }
        if (mode_sec_cnt >= 3)
        {
            mode_sec_cnt = 0;
            Set_Triac_Off();
            mode_step++;
        }
        break;
    case 13:
        // After  0.05sec, turn off motor relay
        if (mode_cnt >= T_52MS)
        {
            F_MOTOR_RELAY = 0;
            mode_step++;
        }
        break;
    case 14:
        // After 0.05sec, turn off CW relay
        // display result and wait for button press
        if (mode_cnt == T_01SEC && mode_sec_cnt == 0)
        {
            F_MOTOR_CW = 0;
            if (test_substep == 1)
            {
                showDirCheck(C_DIRECTION_CW, 'P');
                button_ready = 1;
            }
            else
            {
                showDirCheck(C_DIRECTION_CW, 'F');
            }
        }
        break;

    case 15: // test motor CCW (Spin)
        F_MOTOR_RELAY = 1;
        mode_step++;
        break;
    case 16:
        //After 0.05sec, turn CCW relay on
        if (mode_cnt >= T_52MS)
        {
            F_MOTOR_CCW = 1;
            mode_step++;
        }
        break;
    case 17:
        // After 0.05sec, turn motor triac on and Display CCW
        if (mode_cnt >= T_01SEC)
        {
            showDirCheck(C_DIRECTION_CCW, 0);
            CurrTargetSpeed = C_RPM_SPIN_MIN;
            MotorPIDInit(PID_Kp_CCW, PID_Ki_CCW, PID_Kd_CCW, 0, C_SPEED_PERIOD_SPIN);
            mode_step++;
        }
        break;
    case 18:
        // Run motor for 5.9sec then turn off motor triac.
        if (PID_Time >= C_Inc_PID_Sample_Time)
        {
            if (CurrTargetSpeed < C_RPM_SPIN)
            {
                CurrTargetSpeed += C_RPM_SPIN_INC;
            }
            PID_Time = 0;
            MotorPIDControl(CurrTargetSpeed);
        }
        // Pass if motor Speed between minimum and maximum
        if (test_substep == 0 && CurrentSpeed >= C_MOTOR_CCW_MIN && CurrentSpeed < C_MOTOR_CCW_MAX && mode_sec_cnt >= 1)
        {
            test_substep = 1;
        }
        else if (CurrentSpeed >= C_MOTOR_CCW_MAX)
        {
            test_substep = 0;
        }
        if (mode_sec_cnt >= 6)
        {
            mode_sec_cnt = 0;
            Set_Triac_Off();
            mode_step++;
        }
        break;
    case 19:
        // After 0.05sec, turn off motor relay
        if (mode_cnt >= T_52MS)
        {
            F_MOTOR_RELAY = 0;
            mode_step++;
        }
        break;
    case 20:
        // After 0.05sec, turn off CCW relay
        // display result and wait for button press
        if (mode_cnt == T_01SEC && mode_sec_cnt == 0)
        {
            F_MOTOR_CCW = 0;
            if (test_substep == 1)
            {
                showDirCheck(C_DIRECTION_CCW, 'P');
                button_ready = 1;
            }
            else
            {
                showDirCheck(C_DIRECTION_CCW, 'F');
            }
        }
        break;
    case 21: // finish, go to sleep after 200ms
        if (mode_cnt >= (T_01SEC * 2))
        {
            Go_To_Sleep_Timeout();
        }
        break;
    }
}
#endif
