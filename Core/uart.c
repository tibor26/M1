#include "constant.h"

#ifdef F_DEBUG_EN
#include "stm32g0xx_hal.h"
#include "memory.h"
//**************************************************
static const unsigned char Uart_Cmd_ListAll[] = "Rd";
static const unsigned char Uart_Cmd_SetKp[] = "SetKp";
static const unsigned char Uart_Cmd_SetKi[] = "SetKi";
static const unsigned char Uart_Cmd_SetKd[] = "SetKd";
static const unsigned char Uart_Cmd_ListSpeed[] = "SpRd";
static const unsigned char Uart_Cmd_RealTimeEn[] = "RTEnable";
//*****************************************************************************************
#endif  // include one function to avoid empty translation unit error
void Uart_Debug(void);
#ifdef F_DEBUG_EN
void DebugUart_Receive(void);
void DebugUart_Send(void);

unsigned char Uart_MatchCmd(const unsigned char *pStr);
uint8_t LongData_ASCII(uint32_t data, uint8_t col);
void Uart_PrintMultipleLongData(const uint32_t data[], uint8_t len);
uint8_t ASCII_To_Number(uint8_t col, uint32_t min, uint32_t max, uint32_t *value);
void Message_Err(void);
void Message_Ok(void);
void Uart_PrintLongData(char *pStr, uint32_t ndata);
extern UART_HandleTypeDef huart1;
//*****************************************************************************************
static uint8_t uart_cnt = 0;


void Uart_Debug(void)
{
    DebugUart_Receive();
    DebugUart_Send();
}

void DebugUart_Receive(void)
{
    if (F_UART_RX_PAUSE != 0) return;
    if (F_UART_RX == 0) return;
    F_UART_RX = 0;
    if (Uart_MatchCmd(Uart_Cmd_ListAll) == 1)
    {
        Uart_PrintLongData("P = ", PID_Kp);
        uart_status = 0;
    }
    else if (Uart_MatchCmd(Uart_Cmd_ListSpeed) == 1)
    {
        Uart_PrintLongData("Speed = ", CurrentSpeed);
        uart_status = 50;
    }
    else if (Uart_MatchCmd(Uart_Cmd_RealTimeEn) == 1) //Real Time Debug
    {
        uart_status = 100;
    }
    else if (Uart_MatchCmd(Uart_Cmd_SetKp) == 1)
    {
        uint32_t temp_value;
        if (ASCII_To_Number(5, 0, 0xffff, &temp_value) == 1)
        {
            PID_Kp_CW = temp_value;
            PID_Kp_CCW = temp_value;
            Message_Ok();
        }
        else
        {
            Message_Err();
        }
        uart_status = 200;
    }
    else if (Uart_MatchCmd(Uart_Cmd_SetKi) == 1)
    {
        uint32_t temp_value;
        if (ASCII_To_Number(5, 0, 0xffff, &temp_value) == 1)
        {
            PID_Ki_CW = temp_value;
            PID_Ki_CCW = temp_value;
            Message_Ok();
        }
        else
        {
            Message_Err();
        }
        uart_status = 200;
    }
    else if (Uart_MatchCmd(Uart_Cmd_SetKd) == 1)
    {
        uint32_t temp_value;
        if (ASCII_To_Number(5, 0, 0xffff, &temp_value) == 1)
        {
            PID_Kd_CW = temp_value;
            PID_Kd_CCW = temp_value;
            Message_Ok();
        }
        else
        {
            Message_Err();
        }
        uart_status = 200;
    }
    else
    {
        Message_Err();

        uart_status = 200;
    }
    uart_rx_cnt = 0;      //reset receive counter (IMPORTANT: restart receive)
    F_UART_RX_PAUSE = 1;  // UART receive pause.
    uart_tx_cnt = 0;      //clear count
    F_UART_TX = 1;
    return;
}


void DebugUart_Send(void)
{
    uart_cnt++;
    if (uart_cnt >= T_01SEC)
    {
        uart_cnt = 0;
        if (F_UART_RX_PAUSE != 0)
        {
            if (F_UART_TX == 0)    // Uart transimission is not enable
            {
                switch (uart_status)
                {
                case 0:
                    Uart_PrintLongData("I = ", PID_Ki);
                    uart_status++;
                    break;

                case 1:
                    Uart_PrintLongData("D = ", PID_Kd);
                    uart_status = 200;
                    break;

                case 50:

                    Uart_PrintLongData("Speed = ", CurrentSpeed);
                    uart_status++;
                    break;
                case 51:  // display one empty line for easier reading
                    uart_tx_buf[0] = '\r';
                    uart_tx_buf[1] = '\n';
                    uart_tx_buf[2] = '\0';
                    uart_status = 50;
                    break;

                case 200:  // display one empty line for easier reading
                    uart_tx_buf[0] = '\r';
                    uart_tx_buf[1] = '\n';
                    uart_tx_buf[2] = '\0';
                    uart_status++;
                    break;

                case 201:  // finish rx pause
                    F_UART_RX_PAUSE = 0;
                    break;

                case 100:  // RTEnable
                    {
                        const uint32_t data[6] = {
                            CurrTargetSpeed,
                            CurrentSpeed,
                            PID_Power_Backup,
                            Motor_PhaseAngle,
                            TIM1->CCR1,
                            vibration
                        };
                        Uart_PrintMultipleLongData(data, 6);
                    }
                    break;
                }
                uart_tx_cnt = 0; //clear count
                F_UART_TX = 1;   //set tx flag => start transmit
                uart_rx_cnt = 0;
                F_UART_RX = 0;
            }
        }
    }
    // send uart tx buffer
    if (F_UART_TX == 1 && (USART1->ISR & UART_FLAG_TXFNF))
    {
        if (uart_tx_cnt < L_UART_TX_BUFFER && uart_tx_buf[uart_tx_cnt] != '\0')
        {
            USART1->TDR = uart_tx_buf[uart_tx_cnt];
            uart_tx_cnt++;
        }
        else
        {
            uart_tx_cnt = 0;
            F_UART_TX = 0;
        }
    }
    return;
}
//*************************************************
//* Description: Compare uart command.
//* Arguments  : pStr --- compare string.
//* Returns    : 1 or 0
//*************************************************
unsigned char Uart_MatchCmd(const unsigned char *pStr)
{
    unsigned char index = 0;

    while ((*pStr) != '\0')
    {
        if (uart_rx_buf[index++] != *pStr++)
        {
            return (0);
        }
    }
    return (1);
}


uint8_t LongData_ASCII(uint32_t data, uint8_t col)
{
    if (data <= 99999)  // support up to 5 digits
    {
        if (data > 9)  // handle digits above 10
        {
            uint8_t init_col = col;
            uint16_t decimal = 10000;
            for (uint8_t i = 0; i < 4; i++)
            {
                uint8_t digit = data / decimal;
                // add digit if not zero or digits added before
                if (digit != 0 || col != init_col)
                {
                    uart_tx_buf[col] =  '0' + digit;
                    col++;
                }
                data %= decimal;
                decimal /= 10;
            }
        }
        uart_tx_buf[col] =  '0' + data;
        col++;
    }
    else
    {
        uart_tx_buf[col]     = 'E';
        uart_tx_buf[col + 1] = 'r';
        uart_tx_buf[col + 2] = 'r';
        col += 3;
    }
    return col;
}


//********************************************************
// Parse input ascii data to number
//********************************************************
uint8_t ASCII_To_Number(uint8_t col, uint32_t min, uint32_t max, uint32_t *value)
{
    if (uart_rx_buf[col] != ' ')  // must start with space
    {
        return 0;
    }
    col++;
    *value = 0;
    uint8_t number_pos = 0; //number position
    uint8_t digit = uart_rx_buf[col];
    while ('0' <= digit && digit <= '9')  // digit
    {
        if (number_pos >= 5)  // parse up to 5 digits
        {
            return 0; // number is too long
        }
        *value *= 10;
        *value += digit - '0';
        number_pos++;
        col++;
        digit = uart_rx_buf[col];
    }
    // valid if at least one digit parsed
    // and value in range
    if (number_pos && min <= *value && *value <= max)
    {
        return 1;
    }
    return 0;
}


//************************************
//*
//************************************
void Message_Ok(void)
{
    uart_tx_buf[0] = 'S';
    uart_tx_buf[1] = 'u';
    uart_tx_buf[2] = 'c';
    uart_tx_buf[3] = 'c';
    uart_tx_buf[4] = 'e';
    uart_tx_buf[5] = 'e';
    uart_tx_buf[6] = 'd';
    uart_tx_buf[7] = '!';
    uart_tx_buf[8] = '\r';
    uart_tx_buf[9] = '\n';
    uart_tx_buf[10] = '\0';
}


//************************************
//*
//************************************
void Message_Err(void)
{
    uart_tx_buf[0] = 'C';
    uart_tx_buf[1] = 'm';
    uart_tx_buf[2] = 'd';
    uart_tx_buf[3] = 'E';
    uart_tx_buf[4] = 'r';
    uart_tx_buf[5] = 'r';
    uart_tx_buf[6] = '!';
    uart_tx_buf[7] = '\r';
    uart_tx_buf[8] = '\n';
    uart_tx_buf[9] = '\0';
}


void Uart_PrintLongData(char *pStr, uint32_t ndata)
{
    while (*pStr != '\0') // Null
    {
        if (uart_tx_cnt < (L_UART_TX_BUFFER - 5))
        {
            uart_tx_buf[uart_tx_cnt++] = *pStr;
            pStr++;
        }
        else
        {
            uart_tx_cnt = 0;
            return;
        }
    }

    uart_tx_cnt = LongData_ASCII(ndata, uart_tx_cnt);
    uart_tx_buf[uart_tx_cnt++] = '\r';
    uart_tx_buf[uart_tx_cnt++] = '\n';
    uart_tx_buf[uart_tx_cnt++] = '\0';
    return;
}

/*
 * Print multiple numbers (up to 5 digits)
 * separed by spaces and with CRLN at the end
 * (replaces sprintf)
 *
 **/
void Uart_PrintMultipleLongData(const uint32_t data[], uint8_t len)
{
    uart_tx_cnt = 0;
    for (uint8_t i = 0; i < (len - 1); i++)
    {
        uart_tx_cnt = LongData_ASCII(data[i], uart_tx_cnt);
        uart_tx_buf[uart_tx_cnt] = ' ';
        uart_tx_cnt++;
    }
    uart_tx_cnt = LongData_ASCII(data[len - 1], uart_tx_cnt);
    uart_tx_buf[uart_tx_cnt]     = '\r';
    uart_tx_buf[uart_tx_cnt + 1] = '\n';
    uart_tx_buf[uart_tx_cnt + 2] = '\0';
    uart_tx_cnt += 3;
}

/*
 * Receive UART character
 * and save to buffer
 * Process command when \n received
 **/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (uart_rx_data == '\b')       // Backspace
        {
            if (uart_rx_cnt >= 1)
            {
                uart_rx_buf[uart_rx_cnt - 1] = 0;
                uart_rx_cnt--;
            }
        }
        else if (uart_rx_data == '\n')  // Newline
        {
            uart_rx_buf[uart_rx_cnt] = 0; // to terminate command string
            if (uart_rx_cnt >= 1 && uart_rx_buf[uart_rx_cnt - 1] == '\r')
            {
                uart_rx_buf[uart_rx_cnt - 1] = 0; // clear \r
            }
            uart_rx_cnt = 0; // reset buffer index
            F_UART_RX = 1;   // Process command
        }
        else
        {
            uart_rx_buf[uart_rx_cnt] = uart_rx_data;
            uart_rx_cnt++;
            if (uart_rx_cnt >= L_UART_RX_BUFFER)
            {
                uart_rx_cnt = 0;
            }
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t *)(&uart_rx_data), 1); // Enable IT again.
    }
}

#endif
