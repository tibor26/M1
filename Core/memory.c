#include <stdint.h>
#include "constant.h"

uint8_t mode;
uint8_t mode_step;
uint16_t mode_cnt;
uint8_t mode_sec_cnt;
uint8_t speed_check_cnt;
uint8_t pour_cnt;
uint8_t breathing_word;      // index of word when doing dot matrix breathing
uint8_t breathing_zero_cnt;  // counter for off time when doing led breathing
uint8_t dotmatrix_update_time;  // to signal dotmatrix time display update
volatile uint8_t pulse_step;
uint8_t timed_out_sleep;
uint8_t c_nozzle_debounce;
uint8_t F_NOZZLE_CLOSE;
uint8_t c_speed_debounce;
uint8_t c_speed_debounce_crit;
uint8_t vibration_read_step;
uint8_t vibration_init;
uint16_t vibration;

uint8_t F_DIR, pwmVal;

uint8_t c_interlock_debounce;
uint8_t F_INTERLOCK_CLOSE;

uint8_t F_THERMOSTAT_CLOSE;
volatile uint8_t check_thermostat_cnt;
volatile uint8_t thermostat_pulse_cnt;
uint8_t thermostat_debounce;

uint8_t c_onoff_debounce;
uint8_t F_ONOFF_PRESS;
uint16_t key_hold;

uint32_t timeout_mode;

uint8_t F_MOTOR_RELAY;
volatile uint8_t F_MOTOR_TRIAC;
uint8_t F_MOTOR_CW;
uint8_t F_MOTOR_CCW;

uint8_t exit_code;
uint8_t button_ready;
uint8_t adjust_time, blend_time, spin_time, pour_time;

volatile uint8_t F_TMBASE;
volatile uint8_t ms_cnt;

//AC detection
uint8_t F_AC_DETECT;
volatile uint8_t F_PASS_GUARD;
volatile uint8_t ac_guard_cnt;
volatile uint8_t hz_count;
volatile uint8_t samp_50hz, samp_60hz;
uint8_t ac_base_count;
uint16_t half_ac_cyc;
uint16_t *motor_phase_angle_table;

uint8_t F_CLICK;
uint8_t check_encoderA_cnt, check_encoderB_cnt;
#ifdef F_FW_VERSION_DISPLAY
uint8_t c_fw_version_clicks;
#endif
uint8_t F_ENCODER_A_LOW, F_ENCODER_B_LOW;

//PID
volatile uint8_t PID_Time;
uint16_t CurrTargetSpeed;
uint16_t CurrentSpeed;
uint16_t SpinSpeed;
uint16_t LastSpeed;
uint32_t Hall_Speed_Factor;
uint32_t PID_Kp_CW,PID_Ki_CW,PID_Kd_CW;
uint32_t PID_Kp_CCW,PID_Ki_CCW,PID_Kd_CCW;
int32_t PID_Kp;
int32_t PID_Ki;
int32_t PID_Kd;
uint16_t PID_Power_Backup;
volatile uint16_t Motor_PhaseAngle;
volatile uint16_t Triac_Pulse_Time;
uint16_t Motor_PhaseAngle_Cmd;
uint16_t Triac_Pulse_Time_Cmd;
int32_t PID_Integral;
int32_t PID_Proportional;
int32_t PID_Vib_Integral;

//UART1
#ifdef F_DEBUG_EN
uint8_t uart_tx_cnt;
uint8_t uart_rx_cnt;
uint8_t uart_tx_buf[L_UART_TX_BUFFER];
uint8_t uart_rx_buf[L_UART_RX_BUFFER];
uint8_t uart_status;
volatile uint8_t uart_rx_data;
uint8_t F_UART_TX;
uint8_t F_UART_RX;
uint8_t F_UART_RX_PAUSE;
#endif

//PCBA TEST
#ifdef F_PCBA_TEST
uint8_t test_substep;
#endif
