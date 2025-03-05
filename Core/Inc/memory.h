#include <stdint.h>
#include "constant.h"

extern uint8_t mode;
extern uint8_t mode_step;
extern uint16_t mode_cnt;
extern uint8_t mode_sec_cnt;
extern uint8_t speed_check_cnt;
extern uint8_t pour_cnt;
extern uint8_t breathing_word;      // index of word when doing dot matrix breathing
extern uint8_t breathing_zero_cnt;  // counter for off time when doing led breathing
extern uint8_t dotmatrix_update_time;  // to signal dotmatrix time display update
extern volatile uint8_t pulse_step;
extern uint8_t timed_out_sleep;
extern uint8_t c_nozzle_debounce;
extern uint8_t F_NOZZLE_CLOSE;
extern uint8_t c_speed_debounce;
extern uint8_t c_speed_debounce_crit;
extern uint8_t vibration_read_step;
extern uint8_t vibration_init;
extern uint16_t vibration;

extern uint8_t F_DIR, pwmVal;

extern uint8_t c_interlock_debounce;
extern uint8_t F_INTERLOCK_CLOSE;

extern uint8_t F_THERMOSTAT_CLOSE;
extern volatile uint8_t check_thermostat_cnt;
extern volatile uint8_t thermostat_pulse_cnt;
extern uint8_t thermostat_debounce;

extern uint8_t c_onoff_debounce;
extern uint8_t F_ONOFF_PRESS;
extern uint16_t key_hold;

extern uint32_t timeout_mode;

extern uint8_t F_MOTOR_RELAY;
extern volatile uint8_t F_MOTOR_TRIAC;
extern uint8_t F_MOTOR_CW;
extern uint8_t F_MOTOR_CCW;

extern uint8_t exit_code;
extern uint8_t button_ready;
extern uint8_t adjust_time, blend_time, spin_time, pour_time;

extern volatile uint8_t F_TMBASE;
extern volatile uint8_t ms_cnt;

//AC detection
extern uint8_t F_AC_DETECT;
extern volatile uint8_t F_PASS_GUARD;
extern volatile uint8_t ac_guard_cnt;
extern volatile uint8_t hz_count;
extern volatile uint8_t samp_50hz, samp_60hz;
extern uint8_t ac_base_count;
extern uint16_t half_ac_cyc;
extern uint16_t *motor_phase_angle_table;

extern uint8_t F_CLICK;
extern uint8_t check_encoderA_cnt, check_encoderB_cnt;
#ifdef F_FW_VERSION_DISPLAY
extern uint8_t c_fw_version_clicks;
#endif
extern uint8_t F_ENCODER_A_LOW, F_ENCODER_B_LOW;

//PID
extern volatile uint8_t PID_Time;
extern uint16_t CurrTargetSpeed;
extern uint16_t CurrentSpeed;
extern uint16_t SpinSpeed;
extern uint16_t LastSpeed;
extern uint32_t Hall_Speed_Factor;
extern uint32_t PID_Kp_CW,PID_Ki_CW,PID_Kd_CW;
extern uint32_t PID_Kp_CCW,PID_Ki_CCW,PID_Kd_CCW;
extern int32_t PID_Kp;
extern int32_t PID_Ki;
extern int32_t PID_Kd;
extern uint16_t PID_Power_Backup;
extern volatile uint16_t Motor_PhaseAngle;
extern volatile uint16_t Triac_Pulse_Time;
extern uint16_t Motor_PhaseAngle_Cmd;
extern uint16_t Triac_Pulse_Time_Cmd;
extern int32_t PID_Integral;
extern int32_t PID_Proportional;
extern int32_t PID_Vib_Integral;

//UART1
#ifdef F_DEBUG_EN
extern uint8_t uart_tx_cnt;
extern uint8_t uart_rx_cnt;
extern uint8_t uart_tx_buf[L_UART_TX_BUFFER];
extern uint8_t uart_rx_buf[L_UART_RX_BUFFER];
extern uint8_t uart_status;
extern volatile uint8_t uart_rx_data;
extern uint8_t F_UART_TX;
extern uint8_t F_UART_RX;
extern uint8_t F_UART_RX_PAUSE;
#endif

//PCBA TEST
#ifdef F_PCBA_TEST
extern uint8_t test_substep;
#endif
