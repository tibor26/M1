#include "stm32g0xx_hal.h"
#include "constant.h"
#include "memory.h"
//=========================================================
void Check_Encoder(void);
//=========================================================
void Check_Encoder(void)
{
    if(ENCODER_A_PIN_STATUS() == 0)
    {
        if(F_ENCODER_A_LOW == 0||F_ENCODER_A_LOW == 2)
        {
            check_encoderA_cnt ++;
            if(check_encoderA_cnt >= 2)
            {
                check_encoderA_cnt = 0;
                if(F_ENCODER_B_LOW == 1&&F_ENCODER_A_LOW != 2)
                {
                    F_CLICK = ENCODER_CCW_CLICK;
                }
                F_ENCODER_A_LOW = 1;         // Encoder A output Low level
            }
        }
    }
    else
    {
        if(F_ENCODER_A_LOW == 1||F_ENCODER_A_LOW == 2)
        {
            check_encoderA_cnt ++;
            if(check_encoderA_cnt >= 2)
            {
                check_encoderA_cnt = 0;
                if(F_ENCODER_B_LOW == 0&&F_ENCODER_A_LOW != 2)
                {
                    F_CLICK = ENCODER_CCW_CLICK;
                }
                F_ENCODER_A_LOW = 0;         // Encoder A output High level
            }
        }
    }
    
    if(ENCODER_B_PIN_STATUS() == 0)
    {
        if(F_ENCODER_B_LOW == 0||F_ENCODER_B_LOW == 2)
        {
            check_encoderB_cnt ++;
            if(check_encoderB_cnt >= 2)
            {
                check_encoderB_cnt = 0;
                if(F_ENCODER_A_LOW == 1&&F_ENCODER_B_LOW != 2)        // Encoder A is changed to Low first?
                {
                    F_CLICK = ENCODER_CW_CLICK;
                }
                F_ENCODER_B_LOW = 1;         // Encoder B output Low level
            }
        }
    }
    else
    {
        if(F_ENCODER_B_LOW == 1||F_ENCODER_B_LOW == 2)
        {
            check_encoderB_cnt ++;
            if(check_encoderB_cnt >= 2)
            {
                check_encoderB_cnt = 0;
                if(F_ENCODER_A_LOW == 0&&F_ENCODER_B_LOW != 2)        // Encoder A is changed to High first?
                {
                    F_CLICK = ENCODER_CW_CLICK;
                }
                F_ENCODER_B_LOW = 0;         // Encoder B output High level
            }
        }
    }
}
