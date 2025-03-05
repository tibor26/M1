#include <stdint.h>
#include "constant.h"
#include "memory.h"
//=============================================================
void showTimeOnly(uint8_t thisCount);
void clearDisplay(void);
void showhello(void);
void DisplayFullOn(void);

extern void showString(char *thisString, uint8_t autoCenter, uint8_t fixedFontWidth);
extern void fillPowerTest(uint16_t data);
//==============================================================

void clearDisplay(void)
{
    fillPowerTest(0); // blank display
}

void DisplayFullOn(void)
{
    fillPowerTest(0xFFFF); // full display
}

void showhello(void)
{
    showString("hello", 1, 0);
}

void showCLOSE(void)
{
    showString("CLOSE", 1, 6);
}

void showOPEN(void)
{
    showString("OPEN", 1, 6);
}

void showENJOY(void)
{
    showString("ENJOY", 1, 6);
}

void showLID(void)
{
    showString("LID", 1, 6);
}

void showSPOUT(void)
{
    showString("SPOUT", 1, 6);
}

void showSPIN(void)
{
    showString("SPIN", 1, 6);
}

void showONLY(void)
{
    showString("ONLY", 1, 6);
}

void showSET(void)
{
    showString("SET", 1, 6);
}

void showBLEND(void)
{
    showString("BLEND", 1, 6);
}

void showTIME(void)
{
    showString("TIME", 1, 6);
}

void showTHERM(void)
{
    showString("THERM", 1, 6);
}

void showACHz(void)
{
    showString("AC Hz", 1, 6);
}

void showACCEL(void)
{
    showString("ACCEL", 1, 6);
}

void showERROR(void)
{
    showString("ERROR", 1, 6);
}

void showTimeOnly(uint8_t thisCount)
{
    char time_str[5] = { ' ', ':', 0, 0, 0 };
    if (thisCount >= 60)
    {
        time_str[0] =  '0' + thisCount / 60;
    }
    time_str[2] =  '0' + (thisCount % 60) / 10;
    time_str[3] =  '0' + thisCount % 10;
    showString(time_str, 1, 6);
}

void showNumber(uint32_t number)
{
    char str[7] = { 0 };
    if (number <= 999999)
    {
        uint8_t i = 0;
        if (number > 9)
        {
            uint32_t decimal = 100000;
            for (uint8_t j = 0; j < 5; j++)
            {
                uint8_t digit = number / decimal;
                // add digit if not zero or digits added before
                if (digit != 0 || i != 0)
                {
                    str[i] =  '0' + digit;
                    i++;
                }
                number %= decimal;
                decimal /= 10;
            }
        }
        str[i] = '0' + number;
    }
    else
    {
        str[0] = 'E';
        str[1] = 'R';
        str[2] = 'R';
    }
    showString(str, 1, 6);
}

#ifdef F_PCBA_TEST
void showPASS(void)
{
    showString("PASS", 1, 6);
}

void showFAIL(void)
{
    showString("FAIL", 1, 6);
}

void showDirCheck(uint8_t direction, uint8_t pass)
{
    char mes[] = "CCW P";
    if (pass == 0)
    {
        mes[3] = '\0'; // cut after 3rd char
    }
    else
    {
        mes[4] = pass;
    }
    if (direction == C_DIRECTION_CW)
    {
        showString(mes + 1, 1, 6); // skip first char
    }
    else
    {
        showString(mes, 1, 6);
    }
}

void showAccelCheck(uint8_t pass)
{
    char mes[] = "ACC P";
    mes[4] = pass;
    showString(mes, 1, 6);
}

void showHz(uint8_t ac_detect)
{
    char hz_str[] = "50Hz";
    if (ac_detect == AC_DETECT_60HZ)
    {
        hz_str[0] = '6';
    }
    showString(hz_str, 1, 6);
}

void showInterlockNozzleCheck(uint8_t InterlockNozzle, uint8_t check)
{
    char mes[] = "I:123";
    mes[0] = InterlockNozzle;
    mes[3+check] = '\0'; // cut after 3+ char
    showString(mes, 0, 6);
}
#endif
