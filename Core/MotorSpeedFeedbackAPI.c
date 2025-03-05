#include "constant.h"
#include "memory.h"
#include "BMA400.h"

void MotorPIDControl(uint16_t TargetSpeed);
void Set_Motor_PhaseAngle(uint16_t Power_Level);
void Motor_Thermostat_Init(void);

#if (C_PID_Power_MAX >= PHASE_ANGLE_STEPS)
#error "C_PID_Power_MAX needs to be set lower than PHASE_ANGLE_STEPS"
#endif

const uint16_t PhaseAngle_60Hz_TAB[PHASE_ANGLE_STEPS] =
{
//  0.0%  0.2%  0.4%  0.6%  0.8%  1.0%  1.2%  1.4%  1.6%  1.8%  2.0%
    8033, 7796, 7698, 7622, 7558, 7502, 7451, 7404, 7360, 7319, 7280,
    7243, 7208, 7174, 7142, 7110, 7079, 7049, 7021, 6993, 6965, 6938,
    6913, 6887, 6862, 6837, 6813, 6789, 6766, 6743, 6720, 6698, 6677,
    6655, 6633, 6613, 6592, 6572, 6552, 6532, 6512, 6493, 6473, 6454,
    6435, 6417, 6398, 6380, 6362, 6344, 6326, 6308, 6292, 6274, 6257,
    6240, 6223, 6206, 6189, 6173, 6157, 6140, 6124, 6108, 6093, 6077,
    6061, 6045, 6029, 6014, 5998, 5983, 5968, 5953, 5938, 5923, 5908,
    5894, 5879, 5865, 5850, 5836, 5821, 5807, 5793, 5778, 5764, 5751,
    5737, 5723, 5709, 5695, 5682, 5668, 5654, 5641, 5627, 5613, 5600,
    5587, 5573, 5560, 5548, 5534, 5521, 5508, 5495, 5482, 5469, 5456,
    5443, 5431, 5418, 5405, 5393, 5380, 5368, 5355, 5343, 5330, 5318,
    5305, 5293, 5280, 5268, 5256, 5243, 5231, 5219, 5207, 5194, 5183,
    5171, 5158, 5147, 5134, 5123, 5111, 5098, 5087, 5075, 5063, 5052,
    5040, 5028, 5017, 5005, 4993, 4982, 4970, 4958, 4947, 4935, 4923,
    4913, 4901, 4889, 4878, 4867, 4855, 4843, 4833, 4821, 4809, 4798,
    4787, 4776, 4764, 4753, 4742, 4731, 4719, 4708, 4698, 4686, 4675,
    4663, 4653, 4642, 4631, 4619, 4608, 4598, 4587, 4575, 4564, 4553,
    4543, 4532, 4521, 4509, 4498, 4488, 4477, 4466, 4455, 4444, 4433,
    4423, 4412, 4401, 4390, 4379, 4368, 4358, 4347, 4336, 4325, 4314,
    4303, 4293, 4283, 4272, 4261, 4250, 4239, 4228, 4218, 4208, 4197,
    4186, 4175, 4164, 4153, 4143, 4133, 4122, 4111, 4100, 4090, 4079,
    4068, 4058, 4048, 4037, 4026, 4015, 4005, 3994, 3983, 3973, 3963,
    3952, 3941, 3930, 3920, 3909, 3898, 3888, 3878, 3867, 3856, 3846,
    3835, 3824, 3813, 3803, 3793, 3782, 3771, 3761, 3750, 3739, 3728,
    3718, 3708, 3697, 3686, 3676, 3665, 3654, 3643, 3633, 3623, 3612,
    3601, 3590, 3580, 3569, 3558, 3548, 3537, 3526, 3516, 3505, 3494,
    3483, 3473, 3462, 3451, 3440, 3430, 3419, 3408, 3398, 3387, 3376,
    3365, 3354, 3343, 3333, 3322, 3311, 3300, 3289, 3278, 3268, 3257,
    3246, 3235, 3223, 3213, 3202, 3191, 3180, 3169, 3158, 3147, 3136,
    3125, 3114, 3103, 3092, 3081, 3069, 3058, 3048, 3036, 3025, 3014,
    3003, 2992, 2980, 2969, 2958, 2947, 2935, 2924, 2913, 2901, 2890,
    2878, 2867, 2856, 2844, 2833, 2821, 2810, 2798, 2787, 2775, 2763,
    2752, 2740, 2728, 2717, 2705, 2693, 2682, 2670, 2658, 2646, 2634,
    2623, 2611, 2598, 2587, 2575, 2563, 2551, 2538, 2527, 2514, 2503,
    2490, 2478, 2466, 2453, 2441, 2428, 2416, 2403, 2391, 2378, 2366,
    2353, 2341, 2328, 2316, 2303, 2290, 2277, 2264, 2251, 2238, 2225,
    2213, 2199, 2186, 2173, 2159, 2147, 2133, 2120, 2106, 2093, 2079,
    2066, 2052, 2038, 2024, 2011, 1997, 1983, 1968, 1954, 1940, 1926,
    1912, 1898, 1883, 1868, 1854, 1839, 1824, 1810, 1795, 1780, 1765,
    1750, 1734, 1719, 1703, 1688, 1673, 1657, 1641, 1625, 1609, 1593,
    1577, 1560, 1543, 1527, 1510, 1493, 1477, 1459, 1442, 1424, 1407,
    1389, 1371, 1353, 1335, 1316, 1298, 1279, 1260, 1241, 1221, 1202,
    1182, 1162, 1141, 1120, 1099, 1078, 1057, 1035, 1013, 990,  968,
    944,  920,  896,  872,  847,  821,  795,  768,  741,  713,  683,
    654,  623,  592,  559,  525,  489,  453,  413,  373,  329,  282,
    231,  175,  111
};
// disable end of the table because delays are too small
//                 ,  35,   1,    1                          //100.0%

const uint16_t PhaseAngle_50Hz_TAB[PHASE_ANGLE_STEPS] =
{
    9699, 9415, 9297, 9206, 9130, 9062, 9001, 8945, 8892, 8843, 8796,
    8752, 8710, 8669, 8630, 8592, 8555, 8519, 8485, 8451, 8418, 8386,
    8355, 8324, 8294, 8264, 8235, 8207, 8179, 8151, 8124, 8098, 8072,
    8046, 8020, 7995, 7970, 7946, 7922, 7898, 7874, 7851, 7828, 7805,
    7782, 7760, 7738, 7716, 7694, 7673, 7651, 7630, 7610, 7589, 7568,
    7548, 7527, 7507, 7487, 7467, 7448, 7428, 7409, 7390, 7371, 7352,
    7333, 7314, 7295, 7277, 7258, 7240, 7222, 7204, 7186, 7168, 7150,
    7133, 7115, 7098, 7080, 7063, 7045, 7028, 7011, 6994, 6977, 6961,
    6944, 6927, 6911, 6894, 6878, 6861, 6845, 6829, 6812, 6796, 6780,
    6764, 6748, 6732, 6717, 6701, 6685, 6669, 6654, 6638, 6623, 6607,
    6592, 6577, 6561, 6546, 6531, 6516, 6501, 6486, 6471, 6456, 6441,
    6426, 6411, 6396, 6381, 6367, 6352, 6337, 6323, 6308, 6293, 6279,
    6265, 6250, 6236, 6221, 6207, 6193, 6178, 6164, 6150, 6136, 6122,
    6108, 6094, 6080, 6066, 6052, 6038, 6024, 6010, 5996, 5982, 5968,
    5955, 5941, 5927, 5913, 5900, 5886, 5872, 5859, 5845, 5831, 5818,
    5804, 5791, 5777, 5764, 5750, 5737, 5723, 5710, 5697, 5683, 5670,
    5656, 5643, 5630, 5617, 5603, 5590, 5577, 5564, 5550, 5537, 5524,
    5511, 5498, 5485, 5471, 5458, 5445, 5432, 5419, 5406, 5393, 5380,
    5367, 5354, 5341, 5328, 5315, 5302, 5289, 5276, 5263, 5250, 5237,
    5224, 5211, 5199, 5186, 5173, 5160, 5147, 5134, 5121, 5109, 5096,
    5083, 5070, 5057, 5044, 5032, 5019, 5006, 4993, 4980, 4968, 4955,
    4942, 4929, 4917, 4904, 4891, 4878, 4866, 4853, 4840, 4827, 4815,
    4802, 4789, 4776, 4764, 4751, 4738, 4725, 4713, 4700, 4687, 4675,
    4662, 4649, 4636, 4624, 4611, 4598, 4585, 4573, 4560, 4547, 4534,
    4522, 4509, 4496, 4483, 4471, 4458, 4445, 4432, 4419, 4407, 4394,
    4381, 4368, 4356, 4343, 4330, 4317, 4304, 4291, 4279, 4266, 4253,
    4240, 4227, 4214, 4201, 4188, 4176, 4163, 4150, 4137, 4124, 4111,
    4098, 4085, 4072, 4059, 4046, 4033, 4020, 4007, 3994, 3981, 3968,
    3955, 3942, 3928, 3915, 3902, 3889, 3876, 3863, 3849, 3836, 3823,
    3810, 3797, 3783, 3770, 3757, 3743, 3730, 3717, 3703, 3690, 3677,
    3663, 3650, 3636, 3623, 3609, 3596, 3582, 3569, 3555, 3541, 3528,
    3514, 3500, 3487, 3473, 3459, 3445, 3432, 3418, 3404, 3390, 3376,
    3362, 3348, 3334, 3320, 3306, 3292, 3278, 3264, 3249, 3235, 3221,
    3207, 3193, 3178, 3164, 3150, 3135, 3121, 3106, 3092, 3077, 3063,
    3048, 3033, 3019, 3004, 2989, 2974, 2959, 2944, 2929, 2914, 2899,
    2884, 2869, 2854, 2839, 2823, 2808, 2792, 2777, 2761, 2746, 2730,
    2715, 2699, 2683, 2667, 2651, 2636, 2620, 2604, 2587, 2571, 2555,
    2539, 2522, 2506, 2489, 2473, 2456, 2439, 2422, 2405, 2388, 2371,
    2354, 2337, 2320, 2302, 2285, 2267, 2249, 2232, 2214, 2196, 2178,
    2160, 2141, 2123, 2104, 2086, 2067, 2048, 2029, 2010, 1991, 1971,
    1952, 1932, 1912, 1892, 1872, 1852, 1832, 1811, 1790, 1769, 1748,
    1727, 1705, 1684, 1662, 1639, 1617, 1595, 1572, 1549, 1525, 1502,
    1478, 1454, 1429, 1404, 1379, 1354, 1328, 1302, 1275, 1248, 1221,
    1193, 1164, 1135, 1106, 1076, 1045, 1014, 981,  949,  915,  880,
    845,  808,  770,  731,  690,  647,  603,  556,  507,  455,  398,
    337,  270,  193
};
// disable end of the table because delays are too small
//                    102,  1,    1                          //100.0%


void MotorPIDControl(uint16_t TargetSpeed)
{
    // RPM Calculation:
    // Timer 1 capture compare register contains the time
    // between "speed_period" turns of the motor, timer freq is 100000 Hz
    // "speed_period" turns <-> TIM1->CCR1 * 10 us
    // RPM = (60000000 us * speed_period) / (TIM1->CCR1 * 10 us)
    // RPM = 6000000 * speed_period / TIM1->CCR1
    if (TIM1->CCR1)
    {
        CurrentSpeed = Hall_Speed_Factor / TIM1->CCR1;
    }
    // Alternate method when the motor is too slow to complete "speed_period" turns
    // between PID runs (below 600 RPM for spin, 2400 RPM for blend).
    // We count the number of Hall sensor edges since last run
    // RPM = (1min * number of turns) / period
    // RPM = (60000000 us * (TIM3->CNT / 6)) / (TIM1->CNT * 10 us)
    else if (TIM1->CNT)
    {
        CurrentSpeed = (1000000U * TIM3->CNT) / TIM1->CNT;
        TIM3->CNT = 0; // reset Timer 3 counter
        TIM1->CNT = 0; // reset Timer 1 counter
    }
    else
    {
        CurrentSpeed = 0;
    }

    // check speed for glitches
    if (CurrentSpeed > C_RPM_MAX)
    {
        CurrentSpeed = 0;
    }

    int32_t error = TargetSpeed - CurrentSpeed;

    // Proportional
    PID_Proportional = PID_Kp * error;
    int32_t PID_Pout = PID_Proportional;

    // Integral
    PID_Integral += PID_Ki * error;
    if (PID_Integral > C_PID_Power_MAX*1024)
    {
        PID_Integral = C_PID_Power_MAX*1024;
    }
    else if (PID_Integral < C_PID_Power_MIN*1024)
    {
        PID_Integral = C_PID_Power_MIN*1024;
    }

    // Derivative
#if (F_PI_CONTROLLER != 1)
    int32_t dSpeed = (CurrentSpeed - LastSpeed);
    PID_Pout -= PID_Kd * dSpeed;
    LastSpeed = CurrentSpeed;
#endif

    PID_Pout = (PID_Pout + PID_Integral) / 1024;

    // limit output
    if (PID_Pout > C_PID_Power_MAX)
    {
        PID_Pout = C_PID_Power_MAX;
    }
    else if (PID_Pout < C_PID_Power_MIN)
    {
        PID_Pout = C_PID_Power_MIN;
    }

    Set_Motor_PhaseAngle(PID_Pout);
}


void MotorPIDInit(int32_t kp, int32_t ki, int32_t kd, uint16_t output_init, uint8_t speed_period)
{
    // init speed measurement
    CurrentSpeed = 0;
    Hall_Speed_Factor = 6000000U * speed_period;
    TIM3->ARR = speed_period * 6 - 1; // set Timer 3 to overflow every speed_period turns
    TIM3->CNT = 0; // reset Timer 3 counter
    TIM1->CNT = 0; // reset Timer 1 counter
    TIM1->EGR |= TIM_EVENTSOURCE_UPDATE | TIM_EVENTSOURCE_CC1; // trigger event to reset TIM1->CCR1 and PSC
    TIM1->CR1 |= TIM_CR1_CEN;  // enable Timer 1
#if (F_PI_CONTROLLER != 1)
    LastSpeed = 0;
#endif

    // init PID params and output
    PID_Kp = kp;
    PID_Ki = ki;
    PID_Kd = kd;
    PID_Time = 0;
    if (output_init > C_PID_Power_MAX)
    {
        output_init = C_PID_Power_MAX;
    }
    else if (output_init < C_PID_Power_MIN)
    {
        output_init = C_PID_Power_MIN;
    }
    PID_Integral = output_init * 1024;
    Set_Motor_PhaseAngle(output_init);

    // init speed limit
    c_speed_debounce = 0;
    c_speed_debounce_crit = 0;
    speed_check_cnt = 0; // used to delay speed check

    Motor_Thermostat_Init();

    // turn on AC freq pin interrupt
    SET_BIT(EXTI->FPR1, EXTI_FPR1_FPIF4); // reset flag
    SET_BIT(EXTI->FTSR1, EXTI_FTSR1_FT4); // AC Freq pin
    HAL_NVIC_EnableIRQ(AC_50_60HZ_EXTI_IRQn);

    F_MOTOR_TRIAC = 1;
}


/*
 *  Init thermostat:
 *  reset counters
 *  and enable pin interrupt
 **/
void Motor_Thermostat_Init(void)
{
    // init thermostat detection
    F_THERMOSTAT_CLOSE = 1;
    thermostat_pulse_cnt = 0;
    check_thermostat_cnt = 0;
    thermostat_debounce = 0;

    // turn on thermostat pin interrupt
    SET_BIT(EXTI->RPR1, EXTI_RPR1_RPIF1); // reset flag
    SET_BIT(EXTI->RTSR1, EXTI_RTSR1_RT1); // Thermostat pin
    HAL_NVIC_EnableIRQ(THERMOSTAT_EXTI_IRQn);
}


/*
 *  Set motor phase angle and triac pulse length
 *
 *  The values are first set in cmd variables, then they are applied
 *  in the zero-crossing interrupt and after the first pulse.
 *  So the values are not changed in the middle of a half-cycle.
 *  The pulse length is increased when the phase angle delay is short
 *  so the pulse doesn't end before the holding current is reached.
 */
void Set_Motor_PhaseAngle(uint16_t Power_Level)
{
    Motor_PhaseAngle_Cmd = motor_phase_angle_table[Power_Level];
    if ((Motor_PhaseAngle_Cmd + PHASE_ANGLE_PULSE_HI) >= PHASE_ANGLE_PULSE_MIN)
    {
        Triac_Pulse_Time_Cmd = PHASE_ANGLE_PULSE_HI;
    }
    else
    {
        Triac_Pulse_Time_Cmd = PHASE_ANGLE_PULSE_MIN - Motor_PhaseAngle_Cmd;
    }
    PID_Power_Backup = Power_Level; // for debug
}


/*
 * Turn off Triac and Timer 14 interrupt
 **/
void Set_Triac_Off(void)
{
    F_MOTOR_TRIAC = 0;
    MOTOR_TRIAC_OFF();
    TIM14->DIER = 0; // disable interrupt
    TIM14->CR1 = 0;  // stop timer

    // disable thermostat and AC freq pin interrupts
    HAL_NVIC_DisableIRQ(AC_50_60HZ_EXTI_IRQn);
    HAL_NVIC_DisableIRQ(THERMOSTAT_EXTI_IRQn);
    CLEAR_BIT(EXTI->RTSR1, EXTI_RTSR1_RT1); // Thermostat pin
    CLEAR_BIT(EXTI->FTSR1, EXTI_FTSR1_FT4); // AC Freq pin
}
