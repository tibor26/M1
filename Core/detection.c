#include "stm32g0xx_hal.h"
#include "constant.h"
#include "memory.h"
#include "BMA400.h"
//=========================================================
void Check_Nozzle(void);
void Check_Interlock(void);
void Check_Thermostat(void);
uint8_t Check_Motor_Speed(uint8_t delay_check, uint16_t speed_low, uint16_t speed_high, uint16_t speed_max);
//=========================================================
void Check_Nozzle(void)
{
    if(NOZZLE_PIN_STATUS() == 1)
    {
        if(F_NOZZLE_CLOSE == 1||F_NOZZLE_CLOSE == 2)          // Is nozzle closed?
        {
            c_nozzle_debounce ++;
            if(c_nozzle_debounce >= T_SWITCH_DEBOUNCE)
            {
                c_nozzle_debounce = 0;
                F_NOZZLE_CLOSE = 0;              // Nozzle is open
                timeout_mode = 0;                // Clear timeout counter
            }
        }
        else
        {
            c_nozzle_debounce = 0;
        }
    }
    else
    {
        if(F_NOZZLE_CLOSE == 0||F_NOZZLE_CLOSE == 2)          // Is nozzle open?
        {
            c_nozzle_debounce ++;
            if(c_nozzle_debounce >= T_SWITCH_DEBOUNCE)
            {
                c_nozzle_debounce = 0;
                F_NOZZLE_CLOSE = 1;              // Nozzle is close
                timeout_mode = 0;                // Clear timeout counter
            }
        }
        else
        {
            c_nozzle_debounce = 0;
        }
    }
    return;
}


//******************************************
//* Check Interlock
//******************************************
void Check_Interlock(void)
{
    if(INTERLOCK_PIN_STATUS() == 0)
    {
        if(F_INTERLOCK_CLOSE == 1||F_INTERLOCK_CLOSE == 2)      // Is interlock close?
        {
            c_interlock_debounce ++;
            if(c_interlock_debounce >= T_SWITCH_DEBOUNCE)
            {
                c_interlock_debounce = 0;
                F_INTERLOCK_CLOSE = 0;          // Interlock is open
                timeout_mode = 0;               // Clear timeout counter
                pour_time = DEFAULT_POUR_TIME;  // reset pour time because liquid could be added
                pour_cnt = 0;
            }
        }
        else
        {
            c_interlock_debounce = 0;
        }
        check_thermostat_cnt = 0;
        thermostat_pulse_cnt = 0;
        thermostat_debounce = 0;
    }
    else
    {
        if(F_INTERLOCK_CLOSE == 0||F_INTERLOCK_CLOSE == 2)      // Is interlock open?
        {
            c_interlock_debounce ++;
            if(c_interlock_debounce >= T_SWITCH_DEBOUNCE)
            {
                c_interlock_debounce = 0;
                F_INTERLOCK_CLOSE = 1;          // Interlock is close
                timeout_mode = 0;                // Clear timeout counter
            }
        }
        else
        {
            c_interlock_debounce = 0;
        }
        Check_Thermostat();
    }
    return;
}


//***************************** *
//* Check thermostat is open    *
//***************************** *
void Check_Thermostat(void)
{
    if (check_thermostat_cnt >= L_THERMOSTAT_OPEN)  // check every 80ms
    {
        check_thermostat_cnt = 0;

#ifdef F_THERMOSTAT_EB1
        if (thermostat_pulse_cnt >= 3)
        {
            F_THERMOSTAT_CLOSE = 1; // thermostat close
            thermostat_debounce = 0;
        }
        else
        {
            thermostat_debounce++;
            if (thermostat_debounce >= T_THERMOSTAT_DEBOUNCE)
            {
                F_THERMOSTAT_CLOSE = 0; // thermostat open
            }
        }
#else
        if (thermostat_pulse_cnt >= 3)
        {
            thermostat_debounce++;
            if (thermostat_debounce >= T_THERMOSTAT_DEBOUNCE)
            {
                F_THERMOSTAT_CLOSE = 0; // thermostat open
            }
        }
        else if (THERMOSTAT_PIN_STATUS() == 1)
        {
            F_THERMOSTAT_CLOSE = 1; // thermostat close
            thermostat_debounce = 0;
        }
#endif
        thermostat_pulse_cnt = 0;
    }
    
    return;
}

//***********************
//* Check motor speed  **
//***********************
uint8_t Check_Motor_Speed(uint8_t delay_check, uint16_t speed_low, uint16_t speed_high, uint16_t speed_max)
{
    if (speed_max <= CurrentSpeed)
    {
        c_speed_debounce_crit++;
        if (c_speed_debounce_crit >= T_SPEED_DEBOUNCE_CRIT)
        {
            return 1;  // critical speed, stop motor immediately
        }
    }
    speed_check_cnt++;
    if (speed_check_cnt >= delay_check)
    {
        if (CurrentSpeed <= speed_low || speed_high <= CurrentSpeed)
        {
            c_speed_debounce++;
            if (c_speed_debounce >= T_SPEED_DEBOUNCE)
            {
                return 1;
            }
        }
        else
        {
            c_speed_debounce = 0;
        }
    }
    return 0;
}

/*********************************************************************
 * Vibration controller
 *
 * PI controller to limit the vibration
 * controller target is the vibration limit and output is the speed
 * output is limited by the spin speed target
 *
 *********************************************************************/
uint16_t vibration_controller(uint16_t speed_target)
{
    uint16_t vibration = bma400_read_fifo();

    int32_t error = C_VIBRATION_MAX - vibration;

    // Proportional
    int32_t PID_Vib_Proportional = (int32_t)C_PID_KP_VIB * error;

    // Integral
    PID_Vib_Integral += (int32_t)C_PID_KI_VIB * error;
    if (PID_Vib_Integral > speed_target * 1024)
    {
        PID_Vib_Integral = speed_target * 1024;
    }
    else if (PID_Vib_Integral < C_RPM_SPIN_MIN * 1024)
    {
        PID_Vib_Integral = C_RPM_SPIN_MIN * 1024;
    }

    int32_t speed_out = (PID_Vib_Proportional + PID_Vib_Integral) / 1024;

    // limit output
    if (speed_out > speed_target)
    {
        speed_out = speed_target;
    }
    else if (speed_out < C_RPM_SPIN_MIN)
    {
        speed_out = C_RPM_SPIN_MIN;
    }

    return speed_out;
}


//*******************************
//* Vibration Controller init  **
//*******************************
void vibration_controller_init(uint16_t output_init)
{
    if (output_init > C_RPM_SPIN)
    {
        output_init = C_RPM_SPIN;
    }
    else if (output_init < C_RPM_SPIN_MIN)
    {
        output_init = C_RPM_SPIN_MIN;
    }

    PID_Vib_Integral = output_init * 1024;

    vibration_read_step = 0;
    vibration_init = 0;
    bma400_fifo_flush();
}
