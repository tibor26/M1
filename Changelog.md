# 2025-01-20
### Set up STM32CubeIDE project files
- change .ioc to generate for STM32CubeIDE
- STM32CubeIDE files are in main folder
- set up Release build config to use -O3 optimizations
- add .launch files to git
- remove IAR EWARM folder
- remove VisualGDB .importedproj file

# 2024-12-05
### Fix fix accelerometer SIN high during sleep  (1.6.3)
- sending 0x00 bytes was not working to put MOSI low
- now change pin mode to GPIO output and set it to low
- pin mode is restored to SPI AF in bma400_init()
- version 1.6.3

### Fix accelerometer SIN high during sleep (1.6.2)
- sends 0x00 byte after sleep command to force SPI1 MOSI (PB5 pin 44) to low
- fix sleep mode after PCBA test is finished by going to timeout sleep 200ms after the last button press
- version 1.6.2

# 2024-11-27 (1.6.1)
### Add accelerometer data integrity test
- run only in pcba test mode
- check if all data is not zero
- check if not any axis has all data set to 0xFF
- version 1.6.1

# 2024-10-31
### Fix sleep mode issue with 240V boards
- issue was caused by a drop in 3.3V and 5V when VOLTAGE_SUPPLY was
  turned on on sleep mode exit.
  The drop in voltage was causing flash ECC error or hardfault.
  The drop doesn't happen when the accelerometer CS pin is turned on first
  because it charges the 3.3V rail.
  Another work arround is to enable VOLTAGE_SUPPLY while still in low power mode,
  in this case the voltage still drops but the software doesn't crash.
  This is the chosen fix: turn on VOLTAGE_SUPPLY in low power mode,
  wait 20us for the drop to finish.
- this procedure is also done at the start of the program when VOLTAGE_SUPPLY is first turned on
- leave accelerometer CS pin low after sleep mode init, also low is default state at start
- add wake up 50ms after sleep to do a last check of lid/nozzle pins
- disable button action in sleep mode in key.c, now done in Do_Sleep()
- enable low power mode and remove normal polling sleep mode
- add 2s delay before going to low power mode in case a wake up happens
- wait at least 1ms before accelerometer init after power reset
- add AC signal monitoring when motor is running

# 2024-11-20 (1.5.2)
### Fix accelerometer threshold in self test
- threshold was set for one axis so was triggering sometime since now all 3 axes are enabled
- removed threshold so now it will pass with any amount of vibration
- version 1.5.2

#### Fix thermostat in self test mode (1.5.1)
- thermostat pin interrup was not enabled before the test step
- added a Motor_Thermostat_Init() function
- also changed the interlock/nozzle debounce to 52ms
- fix issue with error mode blink counter reset by lid/nozzle
- version 1.5.1

# 2024-10-28 (1.5.0)
### Improve sleep mode
- work in progress, low power sleep disabled for now using F_SLEEP_LOW_POWER_DISABLE
- issue was when lid/nozzle was closed during the sleep mode init
  the action was missed and the unit still goes to sleep mode
- use EXTI interrupt flags to check if pin state change to fix this
- also check lid/nozzle pin state before going to sleep (except when sleep timeout)
- also reduced the debounce time for lid/nozzle to 20ms
- disable the thermostat and AC Freq pin interrupts when the motor is not active
- UART debug is scheduled every 100ms by the 4ms loop instead of the AC freq
- restore watchdog freq earlier in sleep mode exit
- also changed behavior on interlock open for spin mode, now goes to adjust time
- wake up on button release to prevent starting motor right away
- update version to 1.5.0

# 2024-10-25
### Add improved vibration controller
- PI controller to limit the vibration
- controller target is the vibration limit and output is the speed
- output is limited by SpinSpeed speed target
- spin ramp up sets SpinSpeed then it goes through the vibration controller
  to set CurrTargetSpeed
- if vibration is under the limit then the speed is increased up to the current SpinSpeed
- if vibration is above, the CurrTargetSpeed is decreased
- measure vibration on all axes, vibration limit set to 325 (0.897 g)
- remove 10s slow ramp up, now ramp up in 6s but vibration controller can slow it down
- vibration controller is active after 1s, minimum speed is 700
- improve scheduling of vibration check, vibration data is requested 8ms before PID,
  data is read and speed updated 4ms before PID
- added ACCEL error when the accelerometer is not detected (checked at spin cycle start)

# 2024-10-08 (1.4)
### Change sleep led breathing
- use exp(sin(x)) curve for a more natural breathing effect
- reduce max intensity to 150 (from 248)
- increase period to 3s (from 2.1s)
- also added 1ms delay before reading GPIO on wake up
- update version to C1.4

### Fix issue with version number display in test mode
- the version number display was garbled, the issue was one of the SRAM buffer
  was corrupted so it was displaying the corrupted data and the correct data
  one after the other
- it's possibly caused by the compiler optimization messing up with the order
  of the code
- fixed by declaring SRAMbuffer0Complete as volatile
- also declare all variables which are modified during interrupt as volatile

# 2024-09-24
### Turn off dotmatrix power during sleep
- need new board with voltage control on PB8
- power is cut off from dot matrix, accelerometer, thermostat and AC freq pin
- dot matrix and accel are re initialized after power is back on
- on older boards the power is not cut off but the re init is still done
- fix issue where lid/nozzle state change was not checked during fade out before sleep
- update GDB to version 15.1 rev3 in VisualGDB

### Add AC detect error
- got to error if AC detection is not done after the startup message is done
- AC detection time should be 0.5s and startup message is 4.5s
- display "ERROR" and "AC Hz" words
- fix previous behavior where program was running but without display or motor

# 2024-09-02
### Fix bug when accelerometer is not detected
- wait for SPI BSY was blocking and triggering the watchdog
- don't run bma400_fifo_flush() if accel is not detected
- add timeout for SPI disable so it's not blocking

# 2024-08-28
### Reduce power consumption in sleep mode
- use DMA activated by Timer 17 to do button led breathing effect
- source is a lookup table from 0 to 150 then back to 0 and 30x more 0,
- set breathing period to 1.2s
- reduce clock to 1MHz and go to low power sleep mode
- disable MCO to dotmatrix
- disable periph clocks in sleep mode except for flash, dma and timers 14 and 17
- enable power saving for dot matrix, activated when leds are off
- wait for dot matrix vsync before going to sleep so the power saving is activated
- set accelerometer in sleep mode
- do fade-out transition in software to go from normal led on to off
- lid/nozzle/encoder/button set up as interrupt to wake up from sleep
- timer 14 used to wake up and reset watchdog
- change standby mode name to "adjust time"
- added glitch prevention on speed measurement

### Fix Triac stop
- set triac pin and stop timer immediately when turning of F_MOTOR_TRIAC,
  so it's not turned on again by the timer interrupt

# 2024-08-19
### Improve spin slow ramp up
- skip slow ramp up on resume after pause if already done before
- also skip if doing spin only after a spin cycle
- slow ramp up is reset when lid is open or blend is run (like pour time)

# 2024-08-16 (C1.3)
### Adjust vibration threshold and spin ramp up
- reduce vibration threshold to 75, still using only axis y (vertical)
- new spin ramp up: 10s ramp up from 1000 to 1100 rpm then 5s ramp up to 2500s
- set minimum vibration speed to 1100
- reduce vibration debounce time to 700 ms
- increade spin PID I param to 11
- don't decrease speed the first time vib limit is reached, just stop ramp up
- use version number C1.3 instead of date

### Fix SPI2 BSY
- don't wait for BSY flag for new screen SPI tx, use FTLVL instead

### Fix uart debug set PID params
- number parser was broken because \r was missing from input
- rewrote parser to work without \r
- also removed unused Hex_ASCII() and Uart_PrintData() functions
- reduce RX buffer to 13 to fit longest command "SetKx xxxxx\r\n"

# 2024-08-08
### Fix dotmatrix turn off issue and others fixes
- was caused by uart handling of backspace character,
  when backspace was the first char received it will decrement buffer index two times
  which will underflow and set it to 255. Which corrupted dotmatrix Vsyncbufffer.
- moved HAL_UART_RxCpltCallback() to uart.c
- fix motor relay stayed on when pcba thermostat test fails
- restore calculation of x and z axes accelerometer RMS (but they are still not used)
- increase speed check debounce to 4s and critical speed check to 300ms
- add check if SPI2 is busy before reloading DMA for dotmatrix data
- update version number to C1.2

# 2024-08-02
### Update PCB test mode
- add dot matrix test for only used leds in step 1
- step 0 (test all dotmatrix leds) stays on until button is pressed instead of 3s
- update version number to C1.1
- show "AC ERR" for AC detection timeout
- also increase number of clicks to activate fw version mode to 15 (half a turn)

### Set Vibration detection limit
- only use vertical axis for vibration threshold,
  this seems to be more sensitive to the rocking of the device
- set threshold to 100 (0.276g)
- fix init of accel data reading (in bma400_read_fifo_start())
- reduce RMS window size to 32 samples (320ms)
- rename vibration level variable "std_dev_total" to just "vibration"
- fix vibration limit minimum speed, prevent speed to go under the limit of 1000rpm

### Update VisualGDB toolchain and BSP
- GDB 14.2 rev1 -> 15.1 rev2
- BSP 2024.01 -> 2024.07

# 2024-08-01
### New rpm calculation method
- using Timer 3 and 1 instead of GPIO interrupt
- Timer 3 configured with Hall sensor pin as external clock
  and period set to overflow after a set number of turns
- Timer 1 running at 100000Hz and triggered by Timer 3
  so it captures the time between the set number of turns
- speed measurement period is set to 4 turns for blend and 1 turn for spin
- when the motor is too slow to complete the set number of turns between PID runs,
  the speed is calculated using the number of edges sinces last run (like previous method)
- Spin PID params changed to 50,3 for closer control during ramp up (tested with almonds)
- remove Timer 16 and use Timer 14 for both AC freq detection and Triac control

# 2024-07-25
### Remove unused interrupts in SPI1 accelerometer reading
- disabled SPI1 interrupt
- removed TX DMA (only RX is needed)
- set DMA RX IRQ to just set NSS up (not using HAL)
- initiate SPI1 DMA transfers without using HAL
- changed DMA channels order, channel 1 is SPI1 rx, 2 SPI2 tx
- use dummy SPI transfer instead of HAL delay to wait for the sensor to switch to SPI mode

# 2024-07-22
### Improve vibration calculation
- use Newton's method for square root calculation,
  error is max 2 on std_dev_total compared to math.h sqrt()
- floats and math library are not used anymore
- only check for vibration limit after min speed is reached,
  so the vibration calculation has time to initialize
- changed vibration data window size to 64 (to help divide operations)
- check and reset PID_Time in mode.c instead of MotorPIDControl()

# 2024-07-18
### Fix font data stored to RAM
- tableOfAsciiChars array was missing a second const keyword
- now saved to flash as an initialized array, saving 160 bytes of RAM

# 2024-07-17
### Fix PCBA test mode bug
- motor relay was not turned off after CCW motor test

### Change PID factor scale to improve performance
- change factor from 1000 to 1024 so optimize the divide operation
- PID params increased by 1 for new scale 

# 2024-07-16
### Cleanup PCBA test mode
- move most of logic from key.c, button press only resets led/dotmatrix and advance mode step
- refactor to reuse existing global variables (mode_cnt, mode_step)
- show "ACC F" if accel test fails
- turn on button led after thermostat checks to know when to press the button
- rework display check functions to use parameters instead of one function for each word
- fix indentation
- add comments for each test step

### Fix typo
- change ENCODER_x_CLCIK to ENCODER_x_CLICK

# 2024-07-15
### Improve dot matrix breathing and spin ramp up
- improve dot matrix breathing word switch, switching when brightness reaches 0
- add dot matrix breathing init function
- rename led_cnt to breathing_word since it't only used for dot matrix breathing
- rename mode_cnt1 to mode_sec_cnt since it't only used for multi second counting
- change spin ramp up to increase at 300ms and 700ms which is in sync with the PID time
- move spin ramp up update before PID update so speed follows more closely
- add fixedFontWidth arg to showString() instead of using global variable
- change Error mode to use timeout_mode counter, so we can set all other counter to only 8 bits

# 2024-07-09
### Add option to disable PCBA test mode
- disabled when F_PCBA_TEST is not defined
- change init PID output to 0 for CCW motor test (no overshoot with empty load)
- also update Visual Studio project file

### Add accelerometer test to PCBA test mode
- added after nozzle sensor test
- accel test step is number 4, all following test steps shifted
- test if accel init is sucessful when reading chip id,
  and calculate std dev to see if it's low enough
- std threshold is set 100, normal level is around 30 so it should pass all the time
- use defines in constant.h to set speed target and limits and accel threshold

### Add PCBA test mode added by SMT in version 20240524 (C1.0)
- from the code sent by SMT on 2024-05-30
- SMT code also had changes in mode.c, motor control and others,
  which were the re-implementation of Nama changes,
  only the Nama version is kept
- "W" and "z" letters added in font for test mode messages
- added messages used by test mode in display.c
- cleaned up messages functions in display.c
- added button actions for test mode in key.c
- test mode is started in the pre-main loop which does the ac detect and startup message display
- test mode is activated with 2s long button press at start
- changed firmware version display action to 2s long press to match test mode action
- filename for PCBA test renamed pcba_test.c from test.c
- updated IAR, VisualStudio and STM32CubeIDE project files to add new pcba_test.c file

### Add vibration detection using accelerometer
- add BMA400 sensor driver
- accel using pins SPI1 (PA11, PB3, PB5), NSS PA3, INT1 PA1
- vibration is measured using the sum of the standard deviation data on each axis,
  standard deviation is like the RMS minus the mean,
  when the accelerometer is not moving the std dev should be 0
- speed is decreased when vibration goes above the threshold, speed is increased again when going under the threshold,
  using debounce counters so the speed doesn't oscillate to fast
- vibration data (std_dev_total) is sent on the uart debug in the last number of RTEnable command
- update IAR and STM32Cube project files to add new BMA400 driver file
- if the accel init fails the vibration detection is disabled (vibration stays at 0 and the limit is not reached)
- vibration detection can be disabled using F_VIBRATION_LIMIT_DISABLE
- vibration limit function Check_Vibration() is in detection.c
- also moved speed limit function to detection.c
- vibration threshold set to very high value until we do more tuning, so it doesn't activate for now
- also updated VisualGDB GCC toolchain to 13.3.1

# 2024-07-08
### Move speed check outside of MotorPIDControl()
- speed check is called just after PID control function in mode.c

# 2024-07-05
### Remove fixed stack and heap size in VisualGDB
- causing problems with display going blank
- exact cause still unknown

### Cleanup unused driver files
- remove all .c and .s files from Drivers/CMSIS
- remove unused .c file from Drivers/STM32G0xx_HAL_Driver,
  those are still generated by CubeMX but not needed with the current code
- updated iar and STMCube project files

### Remove printf and malloc usage
- sprintf was used in debug uart, replaced with a custom function
- reworked "int to ascii" function LongData_ASCII() and added function to print multiple numbers Uart_PrintMultipleLongData()
- this remove all usage of malloc and sbrk so the heap would not be used
- seems to fix the issue where the display goes blank caused by the introduction of fixed stack and heap
- fixed stack and heap are still enabled (512 bytes stack, 0 heap)
- cleanup uart.c, fix formatting, remove unnecessary constants

# 2024-07-03
### Increase thermostat detection time
- thermostat is checked every 80ms and is only set as open after 25 consecutive checks (80ms x 25=2s)
- delay can be changed using T_THERMOSTAT_DEBOUNCE in constant.h
- moved thermostat variables init and Triac turn on to MotorPIDInit()
- changed define to switch between EB1 and EB2 thermostat hardware,
  now default is EB2 and EB1 is set when F_THERMOSTAT_EB1 is defined

# 2024-07-01
### Update ioc file with all pin names
- add all pin name in ioc CubeMX file
- update pin write/read defines in constant.h with the pin names
- unused pins are named DEBUG_x with the pin number from the CON10 connector
  (only available on earlier PCB)
- renamed NOZZLE_LED_1 to NOZZLE_LED_EXT (external led on the led sensor board)
- NOZZLE_LED is the test led N on the PCB

### Remove font test mode
- remove font test mode from main branch
- code is kept in a separate branch like the motor test
- also cleaned up unused function Do_01Sec() from main.c

### Fix ":00" display in spin only mode
- when turning down the time very fast :00" was still displayed before "SPIN ONLY"
- this was caused by the dot matrix update delay to avoid glitches
- fixed by canceling the delayed display update when reaching the spin only mode
- also added missing timeout reset on encoder CCW when time is already 0

# 2024-06-28
### Fix motor relays order, cleanup and other fixes
- activate motor and CCW relays after the pouring in spin mode
- added CCW relay step in spin mode
- set the delay to turn off CCW relay to 52ms in spin mode (like blend mode)
- improve behavior when jug is removed (lid/nozzle open) in blend/spin mode,
  go to sleep if the motor has not started yet,
  otherwise do the shutdown procedure and go to sleep in the last step
- changed exit_code step in blend/spin to use "switch" instead of "if else"
- use named mode steps for standby, blend, spin, error and fw version modes
- fix edge case where the spin error could be cancelled if it happened at the end of spin cycle
- remove check before updating phase angle
- rename SHOW_NAMA mode and function to STARTUP_MES
- update comments

# 2024-06-27
### Fix triac control
- issue was when the phase angle was too small,
  the triac pulse was ending before the holding current
- set a minimum triac pulse end time of 1500us after the zero crossing
- cleaned up triac control code now using 4 steps:
  wait phase angle, pulse, wait next half cycle, pulse
- cleanup some code in main.c (remove commented code, fix indentation)
- optimize ac detection, F_AC_DETECT is used to store the detected freq
- added Set_Motor_PhaseAngle() function which sets the phase angle and the pulse time
- removed end of the phase angle lookup tables because values under 100us are not working,
  those can work with special logic to skip step 0 of triac control (triac on all the time)

# 2024-06-20
### Cleanup STM32CubeIDE project folder
- use virtual folders so empty folders in filesystem are not needed
- move syscalls.c to Core/Src
- syscalls.c also add to VisualGDB build to avoid warning on latest GCC versions
- removed sysmem.c
- update STM32CubeIDE linker script to avoid rwx error

# 2024-06-19
### Change startup message to hello
- change nama logo at startup to "hello"
- removed "a", "m", "n" letters from font
- added "h", "e", "l" and "o" in font
- hello letters are 7 height so the "h" an "l" look better

### Optimize dot matrix pixel buffer
- pixel buffer was a 16 bit int array which stored either 0 or 0xffff if pixel was on or off
- changed to 8 bit int array and store 0 or non 0 for pixel on/off
  then shift 0x0000 or 0xffff into display buffer if pixel value is 0 or not
- saves 384 bytes of RAM
- also changed stack size to 512 bytes to match STM32CubeIDE config

### Add STM32CubeIDE project
- used CubeMX and ioc file to generate STM32CubeIDE project
  then added other source and header files as links
- it relies on empty folders in STM32CubeIDE/STM32CubeIDE, this will need to be fixed later
- add STM32CubeIDE folder with project files
- set heap size to 0 and stack size to 512 bytes in ioc file and STM32CubeIDE linker script

# 2024-06-18
### Enable all warnings and fix them
- enabled -Wall -Wextra -Wpedantic
- deleted some unused files to avoid "empty translation unit" warning
  but they are still generarated by CubeMX so they would need to be cleaned up again
- fix other warnings in font.c, uart.c and mode.c
- also set up fixed stack size of 500 bytes, heap size set to 0
  the build will now fail if the static/global variables memory grows too much
- fix uart debug, wait to receive data before parsing command

### Disable UART code when debug uart is not enabled
- use ifdef to disable variables, code, init, interrupt
- saves arround 650 bytes of RAM

### Reduce spacing between letters in nama logo display
- reduce space from 2 colums to 1 column
- lowercase letters are 5 points wide and 5 points high
- one row under letters, two above
- 7 columns on the left, 6 on the right

# 2024-06-12
### Update Cmake config
- fix auto-version so it works when run from another directory

# 2024-06-11
### Add option to disable fw version display mode
- fw version mode is enabled with define F_FW_VERSION_DISPLAY
- when disabled, all the code, variables and defines used by the fw version mode are disabled

# 2024-06-07
### Add firmware version number display
- firmware version mode started by:
    - remove the jug
    - press the button
    - while still keeping the button pressed, rotate dial 10 clicks clockwise
    - while still keeping the button pressed, rotate dial 10 clicks counter-clockwise
- once started it will display the version number (currently the date in 6 digits)
- clockwise dial rotation rotates through the different values: fw version, commit offset, commit hash, commit date and build config
- button press to exit the mode
- it uses CMake to run "git describe" and generate the version.h file (untracked by git)
- if version.h is set by the build system default values for the version number are set in constant.h

### Change VisualGDB project to CMake
- replace MSBuild by CMake
- update MotorSpeedFeedbackAPI new name in IAR project files

# 2024-06-06
### Remove :00 display in spin only mode
- when the time is set to 0 only cycle the words SPIN and ONLY

# 2024-06-04
### Fix PID scheduling
- dot matrix update every second is taking too long (11ms) so a entire Do_Mode() loop is missed
  causing the PID to be ran 4ms late and speed control spikes
- quick fix: schedule PID_time in systick callback independant of Do_Mode() timing
- schedule display update with PID time so it's always run after PID update
- still need to improve dot matrix update performance

# 2024-05-31
### Fix some PID issues, update params and EB2 support
- improve issue where PID update is delayed by dotmatrix update.
  run PID update before dot matrix update so it's less delayed
  advance PID update cycle so it doesn't match with dotmatrix update
  there is still some delay happening so further debug would be needed
- init Integral part to the start output value in PID init function
- init Spin mode to 100 output instead of 0
- update PID params for blend and spin
- change thermostat detection for EB2 hardware (can be configured with F_THERMOSTAT_EB2 define)

# 2024-05-24
### Change spin ramp up period
- total time still 6s
- increase by 100 rpm speed every 1/3 s
- the target speed is always a multiple of 100
- it matches the speed resolution

### Decrease pour time when lid is closed and nozzle open in standby mode
- add counter in standby mode to decrease pour time in background
  when lid is closed and nozzle is open so the spinning can start earlier
- if pour time is non zero but lower than 2s set is to 2s
  so the pour step is not too short
- pour time is reset to 10s everytime the lid is opened
- pour time is reset to 10s in blend mode

### Move fade-in time display to Nama mode
- fade-in time display is only done once at start so no need to be in standby mode
- fade-in if both lid and nozzle are closed
- fade-in in 0.5s
- fade-in not interrupted on encoder/button change
- fade-in stopped on lid/nozzle change to go to standby or sleep

### Leave Enjoy message longer
- changed Enjoy message to stay until 5min timeout
- any change in encoder, lid, nozzle or button will exit and go to sleep or standby
- Enjoy message stays at least 2s during the fade-in effect
- changes in encoder/lid/nozzle/button happening during fade-in will cause an exit as soon as the fade-in is done

### Improve speed limit detection
- check at every speed update (100ms)
- speed check done in motor control function
- add debounce (2=200ms) to critical max speed check
- add "error blend" mode for speed error in blend mode
- increase speed check debounce to 15 (1.5s)
- increase delay before speed check to 5s for blend and 8s for spin

# 2024-05-08
### Fix some bugs
- don't go to sleep immediately when the lid and nozzle are opened and the
  motor is still turning off. Instead wait for the next step
- reset mode_cnt when stopping the motor because of speed error
- redo the pouring step if the lid is opened in spin mode because it means some
  liquid might have been added

# 2024-05-03
### Fix glitches on dotmatrix when adjusting time
- when changing the time only update display when needed
  it prevent some blinking when time is already at min or max and the encoder is turned more
- add 20ms delay between two time updates to avoid glitches
- delay only happens when changing the time too fast and is not noticeable because of all the fast changes on screen

# 2024-05-02
### Add off time when breathing led in sleep mode
- stay off for 120ms between two breathing cycles

# 2024-05-01
### Fix sleep mode exit on lid/nozzle state change
- there was no check if the lid and nozzle were not both open
  so a state change would exit the sleep mode and reenter right away
  causing a blink on the button led
- changed state change detection logic using timeout counter

### Fix timeout in spin mode
- the timeout was not reset when entering spin mode
- if spin mode was entered when the timeout was already close to finished
  it would activate during the wait nozzle open phase

### Improve motor speed check
- function Check_Motor_Speed() in detection.c
- check every 500ms
- max threshold for critical speed (150%), stop the motor immediately
- low and high thresholds (70% and 130%), stop only if seen two consecutive checks
- increased delay before check for spin mode to wait for the ramp up

### Do nama logo fade-in during AC detect
- added nama logo fade-in and AC detect loop before main loop
- no longer delay before nama logo starts to fade in
- main loop doesn't need to check for AC detect since it's done before
- Do_Mode() doesn't check for SHOW_NAMA mode since it's done before
- moved font test to its own infinite loop before the main loop (when enabled)
- removed delay before dotmatrix init which is not needed because of the fade-in brightness

# 2024-04-30
### Add motor spin error detection
- check speed 2s after motor is started and go to error if speed is less than 70% of target
- speed is also checked every seconds after initial 2s delay
- two differents error modes: "error spin" when motor is too slow, "error therm" for thermostat error
- added capital H letter in font for "ERROR THERM" message

# 2024-04-29
### Cleanup code in blend and spin exit step
- remove some redundant check for nozzle/lid

# 2024-04-26
### Allow start spin only with nozzle open
- do not show "close spout" when spin only is selected (time is 00 in standby mode)
- allow pouring phase to start with nozzle open
- nozzle still needs to be closed to be able to change the time to 0 in the first place

# 2024-04-23
### Add fade out of nama logo and fade in of time after
- time display fade in is only done once after nama logo
  and if both lid and nozzle are closed
- fade in is cancelled if encoder is activated
- nama logo time reduced to 4s + 512ms to fade out + 120ms delay after
- fix button led transition when jug is removed
  now it turns off after a small delay when lid/nozzle are opened
  so in case the jug is removed the program goes to sleep mode before

### Fix blinking when entering standby mode
- check nozzle/lid state and only turn off dotmatrix if needed to avoid blinking
- some code cleanup

### Fade in Enjoy message

# 2024-04-22
### Slow down spin ramp up
- start at 700 rpm and increase 60 rpm every 200ms
- ramp up time is 6 seconds

### Fix turn off CCW relay when sleeping
- CCW relay was left on if spining mode timed out and went to sleep
- force turn off all relays when entering sleep mode

### Add breathing dotmatrix for blending and spinning
- breathing on close lid/open spout messages

### Cleanup and small fixes
- removed unused timing defines
- removed F_SHOW_SPIN_ONLY
- reworked timeout function to use only one variable
- removed DelayOff_Motor_CCW() since it's not needed with the brake removed
- cleanup PID code and added limits in PID init functions
- change Do_Error() to reuse led_cnt variable
- removed no_encoder_dial_cnt and use only button ready to do the button 100ms lock after encoder change
- remove conditions on encoder active
- reorder Visual Studio file list
- other code cleanup
- replace char and int by stdint types in some files

# 2024-04-18
### New PID algorithm
- D part can be deactivated by setting F_PI_CONTROLLER to 1 (current default)
- added PID init function to call before blending/spinning to init all params
- use a pointer variable to select the motor phase angle table for 50/60Hz
- update the phase angle tables to include the -300
- add PID params in constants.h
- PID loop time changed to 100ms to sync with RPM measurement

### Changed RPM calculation
- count hall sensor pulses over 100ms instead of using timer capture
- timer 3 is not used anymore
- use pin interrupt to increment counter
- use 100ms counter in PID function

### Changed spinning ramp up
- ramp up speed 100 rpm every 200ms
- starts at 1000 rpm and takes 3s to reach 2500rpm
- init power level to mininum for spin (was 150 before)

### Cleanup unused variables and constants
- deleted unused interrupt.c file
- change types in memory.c to stdint

# 2024-04-05
### Change max blending time to 1 minute

# 2024-04-03
### Breathing on ":00 Spin Only" dislpay
- only "Spin" and "Only" have breathing effect
- ":00" is solid

# 2024-03-27_2
### Breathing Dot Matrix brightness
- using RGB brightness CC_x in FC3 register
- brightness update is scheduled every 4ms after and before pixel update
  which is every 8ms
- used to fade in nama on startup
- dotmatrix is initialised at 0 brighness and is ramped up to max with nama fade in
- "set blend time", "close lid/spout" words fade in/out in standby mode
- other words not done yet

### Change blending done delay to 3 seconds
- 3s seconds leaves time for the motor to stop while not being too long

# 2024-03-27
### Fix Spinning mode end
- program was getting stuck in ENJOY step because of switch case numbering

# 2024-03-26
### Brake removed
- removed switching on the oposite direction relay when stopping
- sequence is now: turn off Triac, Motor relay then direction relay
  (CW for blinding, CCW for spinning)
- for spinning pause, the CCW relay stays active

### Fix button press after motor stop sequence is started
- button press doesn't stop the sequence but set the exit code to ONOFF_PRESS
  so it will go to standby after the motor is stopped

### Cleanup SPI2 config
- removed SPI2 RX dma (dot matrix rx is not used)
- removed SPI2 TX dma interrupt
  It was used to set SPI2TxComplete flag, now the code checks if DMA is complete from register value.
  Don't use HAL to restart SPI2 TX DMA because it relies on transfer complete interrupt.
- update iar EWARM files
- update ioc file


# 2024-02-27
### Spin only mode when time set to 0
- allow adjust_time to go to 0
- show ":00" "SPIN" and "ONLY" when adjust_time is 0 (or ":00" if F_SHOW_SPIN_ONLY is set to 0)
- if adjust_time is 0, button press calls Go_Spinning() instead of blending
- after the button is pressed OPEN SPOUT is displayed and the spinning start after the nozzle is opened

### Add show nama mode
- program starts in show nama mode
- then goes to standby or sleep mode after 5s
- cleaned up Do_Standby() function since show_nama_cnt is removed

### Sleep mode improvements
- move button led breathing to sleep mode
- after "show nama", device goes to sleep if both lid/nozzle are not detected (jug might not be present)
  or to standby instead
- standby mode shows: time, "set blend time", "close lid/spout", "spin only"
- "close lid/spout" is only displayed if at least one is detected, to make sure the jug is present
- if lid and nozzle are both not detected it goes to sleep mode
- delay showing "close lid/spout" by 100ms so it is not shown when removing the jug
  (when removing the jug nozzle sensor is disconnected a bit before lid)
- sleep mode is exited on button/encoder change and also if the nozzle/lid
  state changes from the state is was when entering sleep

### Blend pause when lid removed
- added intitial steps to blend mode: wait lid, delay resume
- when blending is started with button the first 2 steps are skipped
- when lid is removed it goes to step 0 "wait lid" and shows "close lid"

### Spin pause when nozzle is opened
- reworked the spinning mode with "wait lid close" and "wait nozzle open" steps
- removed pre_spinning mode since wait open nozzle is now included in spinning
- spin mode will pause if nozzle is closed and show "open spout"
- also shows "close lid" if lid is open during the pause
- if both lid and nozzle are open it always goes to sleep mode (jug may be removed)

### Show "set blend time"
- show after 15s without encoder change, 1.5s on each word
- doesn't show if "spin only" is active

### Update font
- 5x6 font
- only add used characters
- added font testing function, can be activated with F_FONT_TEST == 1
- font testing code is in display.c: loop of every used word changing every second or on button press

### Cleanup
- remove code for unused leds (nozzle and others)
- remove F_Display_Change
- cleanup duplicate header files

### Add 5s delay after blend
- 00: shown for 5s (1s to stop motor, 4s delay)

### Show ENJOY message for 5s after spinnning

### Turn off button led after blending (when open nozzle is displayed)

### Cancel button only stop motor relay and goes directly to standby if the motor was not running
- instead of going through the motor stop steps
