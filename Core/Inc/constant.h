#include "main.h"
// Version number
// version number should be set automatically by the build system in the version.h file
// those are the fall back values if it's not set
#ifndef VERSION_NUMBER_AUTO
#define VERSION_NUMBER          "1.6.3"  // version number (last git tag)
#define VERSION_OFFSET          "0"       // number of commit since last tag, D appended if state is dirty
#define VERSION_COMMIT          "..."     // git commit hash (6 chars only and uppercase)
#define VERSION_DATE            "..."     // git commit date (6 chars only)
#else
#include "version.h"
#endif
#if   defined(DEBUG)
#define VERSION_BUILD           "DEBUG"   // compiled with -O0
#elif defined(RELEASE)
#define VERSION_BUILD           "RELEAS"  // compiled with -O3
#else
#define VERSION_BUILD           "UNKNOW"
#endif

#define F_FW_VERSION_DISPLAY    1

//Mode definition
#define MODE_SLEEP              0
#define MODE_ADJUST_TIME        1
#define MODE_BLENDING           2
#define MODE_SPINNING           3
#define MODE_STARTUP_MES        4
#define MODE_ERROR              5
#define MODE_PCBA_TEST          6
#ifdef F_FW_VERSION_DISPLAY
#define MODE_FW_VERSION         7
// mode steps for FW version mode
#define MODE_STEP_FW_VERSION_NUMBER  0
#define MODE_STEP_FW_VERSION_OFFSET  1
#define MODE_STEP_FW_VERSION_COMMIT  2
#define MODE_STEP_FW_VERSION_DATE    3
#define MODE_STEP_FW_VERSION_BUILD   4
#endif

#define T_SWITCH_DEBOUNCE       13      // 52ms = 13*4ms
#define T_KEY_BOUNCE            8       // 32ms = 8*4ms
#define T_KEY_HOLD              500     // 2000ms = 500*4ms
#define T_CLICK_FW_VERSION      15      // number of encoder clicks to trigger fw version mode
#define T_SPEED_DEBOUNCE        40      // 4000ms = 40 consecutive 100ms checks
#define T_SPEED_DEBOUNCE_CRIT   3       // 300ms = 3 consecutive 100ms checks
#define T_THERMOSTAT_DEBOUNCE   25      // 2000ms = 25*80ms

#define L_THERMOSTAT_OPEN       20      // 80ms

#define ONOFF_KEY_PIN_STATUS()  HAL_GPIO_ReadPin(ON_OFF_BUTTON_GPIO_Port, ON_OFF_BUTTON_Pin)

// Relay
#define MOTOR_RELAY_ON()        HAL_GPIO_WritePin(MOTOR_RELAY_GPIO_Port, MOTOR_RELAY_Pin, GPIO_PIN_SET)
#define MOTOR_RELAY_OFF()       HAL_GPIO_WritePin(MOTOR_RELAY_GPIO_Port, MOTOR_RELAY_Pin, GPIO_PIN_RESET)

#define MOTOR_TRIAC_ON()        HAL_GPIO_WritePin(TRIAC_CONTROL_GPIO_Port, TRIAC_CONTROL_Pin, GPIO_PIN_SET)
#define MOTOR_TRIAC_OFF()       HAL_GPIO_WritePin(TRIAC_CONTROL_GPIO_Port, TRIAC_CONTROL_Pin, GPIO_PIN_RESET)

#define CW_RELAY_ON()           HAL_GPIO_WritePin(MOTOR_CW_GPIO_Port, MOTOR_CW_Pin, GPIO_PIN_SET)
#define CW_RELAY_OFF()          HAL_GPIO_WritePin(MOTOR_CW_GPIO_Port, MOTOR_CW_Pin, GPIO_PIN_RESET)

#define CCW_RELAY_ON()          HAL_GPIO_WritePin(MOTOR_CCW_GPIO_Port, MOTOR_CCW_Pin, GPIO_PIN_SET)
#define CCW_RELAY_OFF()         HAL_GPIO_WritePin(MOTOR_CCW_GPIO_Port, MOTOR_CCW_Pin, GPIO_PIN_RESET)

//LED
// Dial Arrow LED
#define LED_DIAL_ARROW_ON()     HAL_GPIO_WritePin(DIAL_ARROW_LED_GPIO_Port, DIAL_ARROW_LED_Pin, GPIO_PIN_SET)
#define LED_DIAL_ARROW_OFF()    HAL_GPIO_WritePin(DIAL_ARROW_LED_GPIO_Port, DIAL_ARROW_LED_Pin, GPIO_PIN_RESET)

// Open Nozzle LED
#define LED_NOZZLE_ON()         HAL_GPIO_WritePin(NOZZLE_LED_GPIO_Port, NOZZLE_LED_Pin, GPIO_PIN_SET)
#define LED_NOZZLE_OFF()        HAL_GPIO_WritePin(NOZZLE_LED_GPIO_Port, NOZZLE_LED_Pin, GPIO_PIN_RESET)

// Dispensing LED
#define LED_DISPENSING_ON()     HAL_GPIO_WritePin(DISPENSING_LED_GPIO_Port, DISPENSING_LED_Pin, GPIO_PIN_SET)
#define LED_DISPENSING_OFF()    HAL_GPIO_WritePin(DISPENSING_LED_GPIO_Port, DISPENSING_LED_Pin, GPIO_PIN_RESET)

// Processing LED
#define LED_PROCESSING_ON()     HAL_GPIO_WritePin(PROCESSING_LED_GPIO_Port, PROCESSING_LED_Pin, GPIO_PIN_SET)
#define LED_PROCESSING_OFF()    HAL_GPIO_WritePin(PROCESSING_LED_GPIO_Port, PROCESSING_LED_Pin, GPIO_PIN_RESET)

// Dot matrix, accelerometer, AC Pin, Thermostat voltage supply
#define VOLTAGE_SUPPLY_ON()     HAL_GPIO_WritePin(VOLTAGE_SUPPLY_GPIO_Port, VOLTAGE_SUPPLY_Pin, GPIO_PIN_SET)
#define VOLTAGE_SUPPLY_OFF()    HAL_GPIO_WritePin(VOLTAGE_SUPPLY_GPIO_Port, VOLTAGE_SUPPLY_Pin, GPIO_PIN_RESET)

// Button Text LED
#define LED_BUTTON_SLEEP_STEPS  400
#define LED_BUTTON_PWM_MAX      248
#define LED_BUTTON_TEXT_ON()    TIM17->CCR1 = LED_BUTTON_PWM_MAX
#define LED_BUTTON_TEXT_OFF()   TIM17->CCR1 = 0

//Nozzle
#define NOZZLE_PIN_STATUS()     HAL_GPIO_ReadPin(NOZZLE_INPUT_GPIO_Port, NOZZLE_INPUT_Pin)

//Interlock
#define INTERLOCK_PIN_STATUS()  HAL_GPIO_ReadPin(INTERLOCK_GPIO_Port, INTERLOCK_Pin)

//Encoder
#define ENCODER_A_PIN_STATUS()  HAL_GPIO_ReadPin(ENCODER_CCW_GPIO_Port, ENCODER_CCW_Pin)
#define ENCODER_B_PIN_STATUS()  HAL_GPIO_ReadPin(ENCODER_CW_GPIO_Port, ENCODER_CW_Pin)

//Thermostat
#define THERMOSTAT_PIN_STATUS() HAL_GPIO_ReadPin(THERMOSTAT_GPIO_Port, THERMOSTAT_Pin)

//Time define
#define T_52MS       13         // 52ms = 13 * 4ms
#define T_01SEC      25         // 0.1sec = 25 * 4ms
#define T_05SEC      125        // 0.5sec = 125 * 4ms
#define T_1SEC       250        // 1sec = 250 * 4ms
#define T_1SEC_PID   10         // 1sec in PID loop time count (100ms * 10)
#define T_1_05SEC    375        // 1.5sec = 375 * 4ms
#define T_2SEC       500        // 2sec = 500 * 4ms
#define T_3SEC       750        // 3sec = 750 * 4ms
#define T_5MIN       75000      // 5mins = 75000 * 4ms

#define DELAY_STARTUP_MES       4   // seconds + 512ms to fade out + 120ms delay after
#define DELAY_SET_BLEND_TIME    15  // seconds
#define DELAY_BLEND_RESUME      2   // seconds
#define DELAY_BLEND_DONE        2   // seconds
#define DELAY_BREATHING_OFF     30  // 30*4 = 120 ms  // time spend at 0 brightness when breathing dotmatrix
#define DELAY_CHK_SPEED_BLEND   50  // 50  * 100ms = 5s, check speed after motor start
#define DELAY_CHK_SPEED_SPIN    80  // 80 * 100ms = 8s, longer delay for spin because of ramp up
#define DELAY_DOTMATRIX_TIME    5   // 5*4 = 20ms, delay between two updates of time on dotmatrix to avoid glitches
#define DELAY_SLEEP_CHECK_PINS  50  // wake up 50ms after sleep to do last check of lid/nozzle pin state
#define DELAY_SLEEP_LOW_POWER   2000  // delay before going to low power sleep

#define DEFAULT_POUR_TIME       10  // seconds
#define DEFAULT_POUR_TIME_MIN   2   // minimum pour time (pour time can still be 0)
#define DEFAULT_SPIN_TIME       45  // seconds
#define DEFAULT_BLEND_TIME      60  // seconds

#define L_ADJUST_MAX_TIME       60     // 1min = 60sec
#define L_ADJUST_MIN_TIME       5      // 5sec

// motor operation exit codes
#define EXIT_ONOFF_PRESS         0
#define EXIT_INTERLOCK_OPEN      1
#define EXIT_NOZZLE_OPEN         2
#define EXIT_THERMOSTAT_OPEN     3
#define EXIT_NOZZLE_CLOSE        4
#define EXIT_OPERATION_COMPLETE  5
#define EXIT_MOTOR_SPEED_ERROR   6
#define EXIT_AC_SIGNAL_ERROR     7

// mode steps for sleep mode
#define MODE_STEP_SLEEP_WAIT_DOTMATRIX_SRAM   0
#define MODE_STEP_SLEEP_WAIT_DOTMATRIX_VSYNC  1
#define MODE_STEP_SLEEP_WAIT_LOW_POWER        2
#define MODE_STEP_SLEEP_WAKE_UP               3

// mode steps for adjust time mode
#define MODE_STEP_ADJUST_TIME_CLOSE_MES   0
#define MODE_STEP_ADJUST_TIME_READY       1

// mode steps for blend mode
#define MODE_STEP_BLEND_WAIT_LID      0
#define MODE_STEP_BLEND_DELAY_RESUME  1
#define MODE_STEP_BLEND_DIR_RELAY     2
#define MODE_STEP_BLEND_PID_INIT      3
#define MODE_STEP_BLEND_MOTOR_RUN     4
#define MODE_STEP_BLEND_MOTOR_STOP    5
#define MODE_STEP_BLEND_EXIT_CODE     6
#define MODE_STEP_BLEND_END           7

// mode steps for spin mode
#define MODE_STEP_SPIN_WAIT_LID       0
#define MODE_STEP_SPIN_WAIT_NOZZLE    1
#define MODE_STEP_SPIN_POURING        2
#define MODE_STEP_SPIN_DIR_RELAY      3
#define MODE_STEP_SPIN_PID_INIT       4
#define MODE_STEP_SPIN_MOTOR_RUN      5
#define MODE_STEP_SPIN_MOTOR_STOP     6
#define MODE_STEP_SPIN_EXIT_CODE      7
#define MODE_STEP_SPIN_END            8

// mode steps for error mode
#define MODE_STEP_ERROR_THERMOSTAT    0
#define MODE_STEP_ERROR_SPIN          1
#define MODE_STEP_ERROR_BLEND         2
#define MODE_STEP_ERROR_AC_FREQ       3
#define MODE_STEP_ERROR_ACCEL         4

#define AC_DETECT_NONE             0
#define AC_DETECT_50HZ             5
#define AC_DETECT_60HZ             6
#define PHASE_ANGLE_PULSE_HI       299   // 1us*300 = 300us
#define PHASE_ANGLE_PULSE_MIN      1499  // 1500us, minimum time to end the Triac pulse
                                         // so it doesn't end below the holding current
#define PHASE_ANGLE_STEPS          498   // number of phase angle values in lookup table

#define ENCODER_NO_CLICK        0
#define ENCODER_CW_CLICK        1
#define ENCODER_CCW_CLICK       2

#define F_PI_CONTROLLER         1       // set to 0 to enable D in PID
#define C_Inc_PID_Sample_Time   25      // 4ms * 25 = 100ms, PID loop time is also used to schedule the 1s countdown
#define C_PID_Power_MIN         5       // 1%
#define C_PID_Power_MAX         490     // 98%, needs to be set lower than PHASE_ANGLE_STEPS
#define C_PID_Power_INIT_BLEND  200
#define C_PID_Power_INIT_SPIN   100
#define C_SPEED_PERIOD_BLEND    4       // number of turns to measure the speed for blend
#define C_SPEED_PERIOD_SPIN     1       // number of turns to measure the speed for spin
#define C_RPM_MAX               30000   // max allowed rpm measurement, to prevent glitches
#define C_RPM_BLEND             10000
#define C_RPM_BLEND_LOW         7000    // 70% of normal speed to check for error
#define C_RPM_BLEND_HIGH        13000   // 130% of normal speed to check for error
#define C_RPM_BLEND_MAX         15000   // 150% of normal speed to check for error
#define C_RPM_SPIN_INC          30      // spin speed increment (every 100 ms)
#define C_RPM_SPIN              2500
#define C_RPM_SPIN_MIN          700     // low speed limit to use when not at max target speed
#define C_RPM_SPIN_LOW          1700    // ~70% of normal speed to check for error
#define C_RPM_SPIN_HIGH         3300    // ~130% of normal speed to check for error
#define C_RPM_SPIN_MAX          3700    // ~150% of normal speed to check for error
#define C_VIBRATION_MAX         325     // 325 = 0.898 g, unit is g * 2^(sensor data bits - 2) * sqrt(BMA400_WINDOW_SIZE)
#define C_PID_KP_CW             21
#define C_PID_KI_CW             11
#define C_PID_KD_CW             0
#define C_PID_KP_CCW            50
#define C_PID_KI_CCW            11
#define C_PID_KD_CCW            0
#define C_PID_KP_VIB            300
#define C_PID_KI_VIB            100

//Uart
#define F_DEBUG_EN          1           // debug UART enable or not
#define L_UART_RX_BUFFER    13          // UART RX buffer size, longest command is "SetKx xxxxx\r\n"
#define L_UART_TX_BUFFER    50          // UART TX buffer size

// PCBA test
#define F_PCBA_TEST         1
#define C_MOTOR_CW_TARGET   20000       // speed target for CW motor test
#define C_MOTOR_CW_MIN      19000       // speed to reach to pass CW motor test
#define C_MOTOR_CCW_MIN     2300        // speed min to pass CCW motor test
#define C_MOTOR_CCW_MAX     2800        // speed max to pass CCW motor test
#define C_DIRECTION_CW      0
#define C_DIRECTION_CCW     1

#define DI()                      __disable_irq()
#define EI()                      __enable_irq()
#define CLR_WDT()                 HAL_IWDG_Refresh(&hiwdg)

//#define F_VIBRATION_LIMIT_DISABLE  1

//#define F_THERMOSTAT_EB1    1  // to set thermostat detection for EB1 and earlier
