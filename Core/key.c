#include "stm32g0xx_hal.h"
#include "constant.h"
#include "memory.h"
//=========================================================
void Read_Key(void);
//void Do_Key(void);
void Press_OnOff(void);

extern void Go_To_Blending(void);
extern void Go_To_Spinning(void);
extern void Go_To_Adjust_Time(void);
extern void Go_To_Sleep(void);
extern void clearDisplay(void);
extern void Set_Triac_Off(void);
//=========================================================
/*
void Read_Key(void)
{
    if(ONOFF_KEY_PIN_STATUS() == 1)
    {
        if(F_ONOFF_PRESS == 0)
        {
            c_onoff_debounce ++;
            if(c_onoff_debounce >= T_KEY_BOUNCE)
            {
                c_onoff_debounce = 0;
                F_ONOFF_PRESS = 1;
                F_KEYPRE = 1;
                F_ACTPRE = 0;
                
                //Press_OnOff();
            }
        }
        else
        {
            key_hold ++;
            if(key_hold >= T_KEY_HOLD)
            {
                key_hold = 0;
                F_KEYHLD = 1;
		F_ACTHLD = 0;
                //F_ONOFF_PRESS = 1;
                //Hold_OnOff();
            }
            c_onoff_debounce = 0;
        }
    }
    else
    {
        if(F_ONOFF_PRESS == 1)
        {
            c_onoff_debounce ++;
            if(c_onoff_debounce >= T_KEY_BOUNCE)
            {
                c_onoff_debounce = 0;
                F_ONOFF_PRESS = 0;
                key_hold = 0;
                F_KEYHLD = 0;
                if(F_ACTHLD == 0)F_ACTREL = 1;
                F_ACTHLD = 0;
                F_KEYPRE = 0;
            }
        }
    }
    return;
}
*/
void Read_Key(void)
{
    if(ONOFF_KEY_PIN_STATUS() == 1)
    {
        if(F_ONOFF_PRESS == 0)
        {
            c_onoff_debounce ++;
            if(c_onoff_debounce >= T_KEY_BOUNCE)
            {
                c_onoff_debounce = 0;
                F_ONOFF_PRESS = 1;
                Press_OnOff();
            }
        }
#ifdef F_FW_VERSION_DISPLAY
        else if (key_hold < T_KEY_HOLD)
        {
            key_hold++;  // used to start fw version mode
        }
#endif
    }
    else
    {
        if(F_ONOFF_PRESS == 1)
        {
            c_onoff_debounce ++;
            if(c_onoff_debounce >= T_KEY_BOUNCE)
            {
                c_onoff_debounce = 0;
                F_ONOFF_PRESS = 0;
#ifdef F_FW_VERSION_DISPLAY
                key_hold = 0;             // reset fw version counters
                c_fw_version_clicks = 0;  // when button is not pressed
#endif
            }
        }
    }
    return;
}

//*********************************************
//* Do Key
//*********************************************
/*
void Do_Key(void)
{
  if(F_KEYPRE != 0)
	{
		if(F_ACTPRE == 0)
		{
			F_ACTPRE = 1;
			
			
		}
	}
	else if(F_ACTREL == 1)
	{
		F_ACTREL = 0;
		
		Press_OnOff();
		
	}
	if(F_KEYHLD == 1)
	{
		if(F_ACTHLD == 0)
		{
			F_ACTHLD = 1;
			Hold_OnOff();
                        
		}
	}
	return;
}
*/


void Press_OnOff(void)
{
    if (mode == MODE_ADJUST_TIME)
    {
        if (F_INTERLOCK_CLOSE == 1 && (F_NOZZLE_CLOSE == 1 || adjust_time == 0) && button_ready == 1)
        {
            if (adjust_time != 0)
            {
                Go_To_Blending();
            }
            else  // spin only mode
            {
                Go_To_Spinning();
            }
        }
    }
    else if (mode == MODE_BLENDING)
    {
        if (MODE_STEP_BLEND_DIR_RELAY <= mode_step && mode_step < MODE_STEP_BLEND_MOTOR_STOP)
        {
            Set_Triac_Off(); // Turn off Motor Triac first
            mode_step = MODE_STEP_BLEND_MOTOR_STOP;
            mode_cnt = 0;
            exit_code = EXIT_ONOFF_PRESS;
        }
        else if (mode_step < MODE_STEP_BLEND_DIR_RELAY)
        {
            Go_To_Adjust_Time();
        }
        else  // set exit code to EXIT_ONOFF_PRESS if button is pressed after motor stop sequence is started
        {
            exit_code = EXIT_ONOFF_PRESS;
        }
    }
    else if (mode == MODE_SPINNING)
    {
        if (MODE_STEP_SPIN_DIR_RELAY <= mode_step && mode_step < MODE_STEP_SPIN_MOTOR_STOP)
        {
            Set_Triac_Off(); // Turn off Motor Triac first
            mode_step = MODE_STEP_SPIN_MOTOR_STOP;
            mode_cnt = 0;
            exit_code = EXIT_ONOFF_PRESS;
        }
        else if (mode_step < MODE_STEP_SPIN_DIR_RELAY)
        {
            Go_To_Adjust_Time();
        }
        else  // set exit code to EXIT_ONOFF_PRESS if button is pressed after motor stop sequence is started
        {
            exit_code = EXIT_ONOFF_PRESS;
        }
    }
#ifdef F_PCBA_TEST
    else if (mode == MODE_PCBA_TEST)
    {
        if (button_ready == 1)
        {
            test_substep = 0;
            button_ready = 0;
            mode_cnt = 0;
            mode_sec_cnt = 0;
            LED_DIAL_ARROW_OFF();
            LED_BUTTON_TEXT_OFF();
            clearDisplay();
            mode_step++;
        }
    }
#endif
#ifdef F_FW_VERSION_DISPLAY
    else if (mode == MODE_FW_VERSION)
    {
        Go_To_Sleep();
    }
#endif
    return;
}
