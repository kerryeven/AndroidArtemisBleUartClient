  
/*

TODO: 4/19
  - Need a Big Left and Right 180 deg Turn - 4/19 6:44 almost...need to test
  - Need step_forward to actually move foraward.  
  - If i_speed_left_moving != 0 then set_forward should use old speed and not got through whole incrementing.  Unless it 
    doesn't move with the old value, then should re-increment
  - Would like to keep a scrubber left and right with no movement

   KERobot based on SparkFun Example8_BLE for Artemis Redboard Nano
   Revision 2.7
    - 2.7  - A little more doc and removed some unused functions
    - 2.6  - Got rid of PaulHVA's usermessaging stuff and functions
    - 2.5  - Stop and start WDT in the Forward and Reverse fuunctions to allow for motor rampup time 
              which was exceeding WatchDogTimer timeout and causing re-boot.
    - 2.4  - Added a WatchDog Timer and another CTimer to clear it which runs approximately 1/sec. 
           - Timer2 runs every 2 secs with a WDT timeout same.  Starts WDT after 2 Timer2 isr's.  
           - WDT restart every CNF handleing in amdtps_main.c
    - 2.3  - Added function listing and step_Forward Cleaning function.
    - 2.2  - Add Pump to A3 = pin 3
    - 2.1  - Move to SDK2.4.2 ArdCore 1.0.30 modified wth 2.4.2 SDK files
    - 2.0  - Move to SDK2.3.2 ArdCore 1.0.30
    - 1.3g - Turn left and right on the fly, add speed control faster and slower.
    - 1.3f - moved am_hal_ctimer_clear in ap3_analog.cpp to end of function if (output == AM_HAL_CTIMER_OUTPUT_SECONDARY)
              statement to test if Nano pin 9 pad 39 special handling was causing the pwm problems with viper.  YES it was.
              also, some slight doc changes pretty much the same but not tested yet as 1.3e - core 27
    - 1.3e - servo working now with if servopin...reset timer in core ap3_analog.cpp file.  still using servo.h
              which maps degrees to resolution to calculate time high.
    - 1.3d - Adjust motor speed forward from off to lowest movement right motor only. CHANGED CORE ADDED TIMER 0
              ap3_analog.cpp change caused servo.h to stop working.  Using servoWrite jitters on BLE connectionm
              but will test next with rev 1.3e version which soes not reset timer for servo pwm's.
    - 1.3c - LED_BUILTIN On/Off when connected/disconnected.  Blinks when done booting
           - amdtps_main.c 111, 133 conn open and update - evt callback pointers commented out
               as they were causing cmpiler warnings...never used.
    - 1.3b - Lots of commenting on BLE and took out some button stuff in the amdtp files
    - 1.3a - Added ino AdvName string conversion to ble advertising data
    - 1.3  - Added first pass of encoder speed control.  Changed chips "name" to REV_1_3
    - 1.2  - Commented well and stopped distance measurements on stop
    - 1.1  - Working Scrub Motor, Servo, Drive Motors, HC-S3R04 UltraSonic Distance Sensor, 7.2v Batt Pack

   NOTES:
    - HAD TO MODIFY SF CORE ap3_analog.cpp create line 360:
        am_hal_ctimer_clear(timer, segment); // clear timer and reset to 0  //KHE ADDED
        Symptom: was inverting voltage on second ramp up from 0 ... consistently and on board with nothing attached but
          with 3 total pwm's to motors, 1 servo, and 1 ultrasonic sensor.
    - Allowing the motor control pwm pins to go to zero results in unexpected
      responses...ie: tracks wone way or another when it should go straight.
      USE THE HIGH/LOW control pins both equal to stop the motors, not analogWrite)pin,0);

    - HC-SR04 Sensors can run on common trigger pin from either 3.3v pin or Logic Level Shifter.
      Echo pin must run directly to 3.3v on the Nano.  It will not function through the shifter
      and I have not seen any voltages above 3.3 vlts output from this particular sensor.  Amazon
      ELEGOO 5PCS HC-SR04 Ultrasonic Module Distance Sensor for Arduino UNO MEGA2560 Nano Robot XBee ZigBee

  TODO:
    - Verify SDK 2.3.2 fixes need to am_hal_ctimer_clear to fix PWM problems
    - Set i_Moving to 0 in set_stop to stop measuing distance when not moving
    - Logic to increase speed (pwm) if wheel not turning.
    - Change i_StopDistance from wall logic to sunroom roof logic
    Done - by incorporating rev in name - Print Arduino revision to phone when connected
    DONE - Turn LED on when connected...off when not connected
    DONE - Remove comments b4 i_Moving = 0; line in set_Stop fn
    DONE - Fix wiring to HC-S04 sonic distance sensors.
    DONE- Turn motors etc. off when lose connection. Got it .. in amdtp_main.c amdtpClose function added
        RobotDisconnectd so far defined i BLE_example_funcs.cpp and just prints disconnectedx messwage to serial
    DONE - Mount scrubber motor and make certain you have enough juice
    DONE - Use PWM for scrubber motor? - yes 300...100 stalls on start
    DONE - Move the distance return to phone to within the < if statement to reduce commands during normal operation
           when done playing
    DONE - Decide on servo nubers for up and down = use degrees
    DONE - Decide if printing distance to phone takes too much time and vinder vaasher runs into wall or off roof
    Done - Send distance measured to phone:
    Done - Only do distance check when moving.
*/
// maximum length of reply / data message
#define MAXREPLY 100
// buffer to reply to client
uint8_t  val[MAXREPLY];
uint8_t  *val_data = &val[2];   // start of optional data area
uint8_t  *val_len  = &val[1];   // store length of optional data
/********************************************************************************************************************
                 INCLUDES
 *******************************************************************************************************************/
#include "BLE_example.h"
#include <Servo.h> //Have to use the Servo.h included in Sparkfun core for this board to work
/********************************************************************************************************************
                 OBJECTS
 *******************************************************************************************************************/
Servo servo; //Create a servo object
/********************************************************************************************************************
                GLOBAL VARIABLES
 *******************************************************************************************************************/
String s_Rev = "Rev 2.7";
String s_Rcvd = "100"; //KHE declare extern in BLE_example_funcs.cpp to accept messages, if 100 don't bother checking case statements
String s_AdvName = "ViperBLE_Rev-2_7"; //KHE 2 0 TOTAL CHARACHTERS ONLY!!  any more will be dropped
int i_B4Timer2_LooperCount = 100;
/********************************************************************************************************************
                Enq Variable
 *******************************************************************************************************************/
//e = 101 dec. = 0x65, n = 110 = 0x6e, q = 113 = 0x71
uint8_t valEnq [] = {(byte)0x65,(byte)0x6e,(byte)0x71};
// *********************************************************************
// Global variables WDT
// *********************************************************************
volatile uint8_t watchdogCounter = 0; // Watchdog interrupt counter
uint32_t resetStatus = 0; // Reset status register

// Watchdog timer configuration structure.
am_hal_wdt_config_t g_sWatchdogConfig = { //g_sWatchdogConfig.ui16InterruptCount = 500 ui16ResetCount

  // Configuration values for generated watchdog timer event.
  .ui32Config = AM_HAL_WDT_LFRC_CLK_16HZ | AM_HAL_WDT_ENABLE_RESET | AM_HAL_WDT_ENABLE_INTERRUPT,

  // Number of watchdog timer ticks allowed before a watchdog interrupt event is generated.
  .ui16InterruptCount = 93, // 20 Set WDT interrupt timeout for 10 seconds (80 / 16 = 5).

  // Number of watchdog timer ticks allowed before the watchdog will issue a system reset.
  .ui16ResetCount =  93 // 60 Set WDT reset timeout for 15 seconds (240 / 16 = 15). was 240
};

/********************************************************************************************************************
                Timer2 VARIABLES
 *******************************************************************************************************************/
static int myTimer = 2;
static int blinkPin = LED_BUILTIN;

int count = 0;


/********************************************************************************************************************
                 GLOBAL HC-SR04 VARIABLES
 *******************************************************************************************************************/
const int trigPin = 8; //Common trigger pin for HC-SR04 Distance Sensor - can be logic level shifted
const int echoPin7 = 7; //Echo pin cannot use logic level shifter.  It outputs < 3.3v and is too low with shifting
int duration7; //khe ori = float was causing troubles - don't need precision
int distance7; //KHE ori = float was causing troubles - don't need precision
long l_Timer = 0; //Incremented variable used to limit distance checking call in Loop function to about 1/sec or so
long l_Count = 0;
int i_stopDistance = 50; //Stop movement if distance < these cm's.
int i_Moving = 0; //Used to for distance checking if 1 = check, not if 0
/********************************************************************************************************************
                 MOTOR VARIABLES
 *******************************************************************************************************************/
double i_Speed = 0; // Use the same speed for both motors
static int i_currSpeedRight = 0;
static int i_currSpeedLeft = 0;
double i_Speed_Left_Moving = 0; //Used as reference
double i_Speed_Right_Moving = 0;
double i_Speed_Left = 0;
double i_Speed_Right = 0;
double i_StartSpeed = 20000; //Lesser pwm values = motor hums wth no movement
//found following factor by trying diff numbers for i_StartSpeedLeft till drove in straight line
// = 185 vs 155 for right motor so 185/155 = 1.1935
double i_Speed_Left_Factor = 1.1935;  //i_int = (int)f_float;  = convert float to int.
static int32_t i_StartSpeedRight = 6000; //20000, 12000 too fast
static int32_t i_StartSpeedLeft = 7000; //24000 Turning a little left when set same as right so increase ~ 17%
int i_MotorPinLeft = 9; //9
int i_MotorPinRight = 14; //14
int i_Inp1 = 10; //HI LO digital //1 & 2 same = off, 1HI-2LO = FWD, 1LO-2HI = BCK
int i_Inp2 = 11; //HI LO digital
int i_Inp3 = 12;
int i_Inp4 = 13;
int i_ScrubMotorPin = 15;
int i_SgeeServoPin = 16;
int i_Pump_Pin = 3; //Pump Digital pin
int i_ServoPos = 0;

/********************************************************************************************************************
                 MOTOR Encoder VARIABLES - no speed implementation yet
 *******************************************************************************************************************/
int i_EncPinLeft = 6; //Left encoder input pin
int i_EncPinRight = 5; //Right Motor encoder input pin 5
int i_CountEncRight = 0; // Right Motor encoder count
int i_CountEncLeft = 0;
bool interruptsEnabled = false;

/********************************************************************************************************************
   JUMP
    Functions:
      void setup()
      int check_SpeedRight()              :Starts right encoder interrupt and counts for 250 MSec returns i_CountEncRight
      int check_SpeedLeft()               :       left                                            returns i_CountEncLeft
      void scrub_Right()                  :Turn pump on, move right and scrub for 1 sec
      void scrub_Left()                   :Turn pump on, move leftt and scrub for 1 sec
      void step_Forward(int i_numCounts)  : Steps forward # of encoder counts then sweeps left and right with motor on then motor off
      int check_SpeedRight()              : Records encoder counts on right motor for 250ms and returns # in i_CountEncRight
      int check_SpeedLeft()               : Records encoder counts on left motor for 250ms and returns # in i_CountEncLeft
      void myISRRight(void)               : Responsds to the ecoder isr incrementing i_CountEncRight
      void myISRLeft(void)                : Responsds to the ecoder isr incrementing i_CountEncLeft
      void set_ScrubMotor( int i_OnOff)   : called with 0 for off or 60000 for on
      void set_SgServo( int i_OnOff)      : Raise Squeegee = 0, Lower = 90
      void set_Stop( void )               : Stops both motors with analogWrites 0, and setting controller board pins all high
      void set_Right( int i_Control )     : i_Control = 1 = Sets motor control pins, then analogWrite 40000 for 150ms
                                            i_Control = 2 = Adds 1000 to pwm for right motor every time button is hit turning
                                            right while continuing to move forward.
      void set_Left( int i_Control )      : same as set_Right except left motor and left turn.
      void set_Faster( void )             : Adds 500 to pwm then analogWrites to motor pins
      void set_Slower( void )             : Subtracts 500 to pwm then analogWrites to motor pins
      void scrub_Right()                  : Turns pump on, turns small right, brush motor on high (60000) and runs for 2 sec's .
      void scrub_Left()                   :                            left
      void step_Forward(int i_numCounts)  : Moves forward for number of right encoder counts - uses speed set in set_Forward
      void set_Forward( void )            : Starts with pwm at 0, increase left and right by 1000 and check_SpeedRight & Left
                                            to sense movement. Continues if either left or right = 0 and left < 60000 (max is 65535)
                                            Adds 2000 to final number just to get things moving nicely and stores value in i_Speed_Left_Moving
                                            and i_Speed_Right_Moving for use in step_Forward
      void set_Reverse( void )            : same as forward using reverse motor control board pin config.
      void check_Distance(void)           : N/A Distance sensor stuff turned on if i_Moving = 1 which coult be set in set_Forward and set_Reverse
      void set_Pump(int i_PumpOnOff)      : Turns on water pump with 1 and off with 0
      void loop()                         : Switch / Case statements for values received from client (phone) app and check_Distance timed calls
      void setupWdt()                     : Setup WatchDogTimer (WDT) - reboots if timer2 gets to 0 
                                            - Details in BLE-example_funcs.cpp scheduler_timer_init function.
      extern void am_watchdog_isr(void)   : Interrupt for WatchDogTimer - print message RE-BOOTING
      extern void timer_isr(void)         : Timer2 isr - sends e n q (HEX) to phone which replies with ACK handled in amdtps_main.c CNF function
                                              to reset the WDT
      void trigger_timers()               : Set/Check main timers/interrupts - allows sleeping/waking properly
      


 *******************************************************************************************************************/

/*************************************************************************************************/
/*!
    \fn     setup

    \brief  Arduino setup function.  Set up the board BLE and GPIO - BLE first...

    \param  none

    \called Arduino startup

    \return None.
*/
/*************************************************************************************************/
void setup() {
#ifdef BLE_SHOW_DATA
    //SERIAL_PORT.begin(115200);
    //delay(1000);
    //SERIAL_PORT.printf("Viper. Compiled: %s\n" , __TIME__);
#endif

#ifdef AM_DEBUG_PRINTF
 //
  // Enable printing to the console.
  //
  enable_print_interface();
#endif

  Serial.begin(115200);
  delay(1000); 
  Serial.printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"); 
  Serial.print("Revision = ");
  Serial.print(s_Rev);
  Serial.printf("  VIPER. Compiled: %s   %s\n", __DATE__,__TIME__);
  Serial.printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"); 
  analogWriteResolution(16); //Set AnalogWrite resolution to 16 bit = 0 - 65535 (but make max 64k or trouble)
  analogWrite(i_MotorPinLeft, 0);
  analogWrite(i_MotorPinRight, 0);

  /********************************************************************************************************************
                  Set Advertising name:  uses global string s_AdvName set above.
   *******************************************************************************************************************/
  set_Adv_Name(); //BLE_example_funcs.cpp near end of file, fn declared extern in BLE_example.h

  /********************************************************************************************************************
                   Boot the radio
                    SDK/third_party/exactle/sw/hci/apollo3/hci_drv_apollo3.c
                    = huge program to handle the ble radio stuff in this file
   *******************************************************************************************************************/
  HciDrvRadioBoot(0);

  /************************************************************************************************
        Initialize the main ExactLE stack: BLE_example_funcs.cpp
        - One time timer
        - timer for handler
        - dynamic memory buffer
        - security
        - HCI host conroller interface
        - DM device manager software
        - L2CAP data transfer management
        - ATT - Low Energy handlers
        - SMP - Low Energy security
        - APP - application handlers..global settings..etc
        - NUS - nordic location services

   ************************************************************************************************/
  exactle_stack_init();

  /*************************************************************************************************
      Set the power level to it's maximum of 15 decimal...defined in hci_drv_apollo3.h as 0xF
      needs to come after the HCI stack is initialized in previous line
        - poss. levels = 0x03=-20,0x04=-10,0x05=-5,0x08=0,0x0F=4 but have to use definitions, not these ints
          extremes make a difference of about 10 at 1 foot.
   ************************************************************************************************/
  HciVsA3_SetRfPowerLevelEx(TX_POWER_LEVEL_PLUS_3P0_dBm); //= 15 decimal = max power WORKS..default = 0

  /*************************************************************************************************
      Start the "Amdtp" (AmbiqMicro Data Transfer Protocol) profile. Function in amdtp_main.c

       Register for stack callbacks
       - Register callback with DM for scan and advertising events with security
       - Register callback with Connection Manager with client id
       - Register callback with ATT low energy handlers
       - Register callback with ATT low enerty connection handlers
       - Register callback with ATT CCC = client charachteristic configuration array
       - Register for app framework discovery callbacks
       - Initialize attribute server database
       - Reset the device

   ************************************************************************************************/
  AmdtpStart();

  /*************************************************************************************************
     On first boot after upload and boot from battery, pwm on pin 14 not working
      need to reset nano board several times with battery power applied to get
      working.  Delay 5 seconds works..haven't tried lesser values.
   ************************************************************************************************/
  delay(5000);

  /************************************************************************************************
      Arduino device GPIO control setup.
        Place after board BLE setup stuff happens.  ie.:
          could not get A14 to PWM untill I moved the set_stop() call from the
          beginning of setup to this location...then works great.
   ************************************************************************************************/


  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(i_MotorPinLeft, OUTPUT);
  pinMode(i_MotorPinRight, OUTPUT);
  pinMode(i_ScrubMotorPin, OUTPUT);
  pinMode(i_Inp1, OUTPUT);
  pinMode(i_Inp2, OUTPUT);
  pinMode(i_Inp3, OUTPUT);
  pinMode(i_Inp4, OUTPUT);
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, 0); //TrigPin can go through Logic Level Shifting 3.3 to 5v, only need 1 trigger for multiple HC-S04's
  pinMode(echoPin7, INPUT); //Tried INPUT_PULLDOWN = 12.54-13.04, INPUT & INPUT_PULLUP 12.52-12.61
  pinMode(i_Pump_Pin, OUTPUT); //Did not work without pinMode - produced tiny voltage at HIGH

  //Set a starting point...for motors, servo, and LED_BUILTIN
  set_Stop();
  i_Speed = i_StartSpeed;
  i_Speed_Left = i_StartSpeedLeft; //30 higher to accomadate practical motor difference
  i_Speed_Right = i_StartSpeedRight;
  set_ScrubMotor(0);
  set_SgServo(0); //1 will swing to 45 then to 0 //nice to see it move on startup
  delay(1000);
  set_SgServo(90); //1 will swing to 45 then to 0 //nice to see it move on startup
  for (int i = 0; i < 20; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  }

  //analogWrite(i_MotorPinLeft, i_Speed_Left);
  //analogWrite(i_MotorPinRight, i_Speed_Right);
  //Serial.print("Left Motor Speed = ");
  //Serial.print(i_Speed_Left);
  //Serial.print("   Right Motor Speed = ");
  //Serial.println("Done with Setup");

  pinMode(i_EncPinRight, INPUT_PULLUP);
  pinMode(i_EncPinLeft, INPUT_PULLUP);

  interruptsEnabled = false;

  //setupwdt();

  pinMode(blinkPin, OUTPUT);
  digitalWrite(blinkPin, LOW);

  // Configure the watchdog.
  //setupTimerA(myTimer, 31); // timerNum, period - //moved to BLE_example_funcs.cpp scheduler_timer_init
  setupWdt();
  am_hal_wdt_init(&g_sWatchdogConfig); 
  //NVIC_EnableIRQ(CTIMER_IRQn); // Enable CTIMER interrupt in nested vector interrupt controller.
  NVIC_EnableIRQ(WDT_IRQn); // Enable WDT interrupt in nested vector interrupt controller.

  am_hal_interrupt_master_enable();
  //interrupts(); // Enable interrupt operation. Equivalent to am_hal_rtc_int_enable().
  //am_hal_wdt_start();
  //am_hal_wdt_int_enable(); - freezes boot
} /*** END setup FCN ***/

int check_SpeedRight() {
  i_CountEncRight = 0;
  attachInterrupt(digitalPinToInterrupt(i_EncPinRight), myISRRight, RISING);
  interruptsEnabled = true;
  delay(250);
  detachInterrupt(digitalPinToInterrupt(i_EncPinRight));
  interruptsEnabled = false;
  Serial.print(" i_CountEncRight = ");
  Serial.println(i_CountEncRight);
  return i_CountEncRight;
}

int check_SpeedLeft() {
  i_CountEncLeft = 0;
  attachInterrupt(digitalPinToInterrupt(i_EncPinLeft), myISRLeft, RISING);
  interruptsEnabled = true;
  delay(250);
  detachInterrupt(digitalPinToInterrupt(i_EncPinLeft));
  interruptsEnabled = false;
  Serial.print(" i_CountEncLeft = ");
  Serial.println(i_CountEncLeft);
  return i_CountEncLeft;
}

void myISRRight(void)
{
  //  Serial.print("Hi i am an ISR! count = ");
  //  Serial.println(count);
  i_CountEncRight ++;
}
void myISRLeft(void)
{
  //  Serial.print("Hi i am an ISR! count = ");
  //  Serial.println(count);
  i_CountEncLeft ++;
}

/*************************************************************************************************/
/*!
    \fn     set_ScrubMotor

    \brief  Front motor used to turn on and off the scrb brush / roller (TBD).  Uses Mosfet transistor
              with pin 15 as PWM gate to neg. drain.  .1uf Capacitor across motor +/- terminals.

    \param  i_OnOff    Int 0 for off or 60000 for on

    \called Android -> Amdtps_main.c fn = amdtps_write_cback
                    -> BLE_example_funcs.cpp  fn = bleRxTxReceived sets amdtpsNano extern String s_Rcvd
                       with char of uint8_t byte array Android Command which is checked in
                       amdtpsNano.ino Loop function Switch statement if s_Rcvd has changed from 100.

    \return None.
*/
/*************************************************************************************************/

void set_ScrubMotor( int i_OnOff) { //called with 0 for off or 60000 for on
  s_Rcvd = "100"; //Set s_Rcvd so it won't be called repeatedly
  analogWrite(i_ScrubMotorPin, i_OnOff);
} /** END set_ScrubMotor FN ***/

/*************************************************************************************************/
/*!
    \fn     set_SgServo

    \brief  Servo Motor raises and lowers Squeegee arm for the Vinder Viper

    \param  i_OnOff    Int 0 for UP or 1 for down

    \called Android -> Amdtps_main.c fn = amdtps_write_cback
                    -> BLE_example_funcs.cpp  fn = bleRxTxReceived sets amdtpsNano extern String s_Rcvd
                       with char of uint8_t byte array Android Command which is checked in
                       amdtpsNano.ino Loop function Switch statement if s_Rcvd has changed from 100.

    \Notes  Must be attached / moved / detached within function or odd behavior occurs.

    \return None.
*/
/*************************************************************************************************/
void set_SgServo( int i_OnOff) {
  s_Rcvd = "100"; //Set this to 100 to ignore switch statement in loop till new msg arrives
  //SparkFun Core servo.cpp servo.h simply maps degrees or MicroSecs to a value based on whatever resolution
  //  the servoWriteResolution is set to with next two lines.  Then ap3_analog.cpp servoWrite is called.
  //    uint16_t newServoPosition = map(microSecs or position, extendedMin, extendedMax + 1, 0, ((uint16_t)0x01 << getServoResolution()));
  //    servoWrite(_servoPinNumber, newServoPosition, extendedMin, extendedMax);
  //  servo.attach just sets the min and max microsecond pulse and sets pinMode to INPUT.  detach sets to OUTPUT.
  //  servo resolution is 8 by default (0 - 255);

  servo.attach(i_SgeeServoPin, 544, 2400); //Have to attach | move | detach in one action or jitters
  if (i_OnOff != 90 || i_OnOff == 0) { //Move to 0 (UP)
    Serial.println("Servo to 0");
    for ( int i = 90; i >= 0; i--) {
      //servoWrite(i_SgeeServoPin,i);
      servo.write(i);
      delay(10);
    }
  }
  if (i_OnOff != 0 || i_OnOff == 90) { //Move to 90 (DOWN)
    Serial.println("Servo to 90");
    for (int i = 0; i <= 90; i++) { //Move from 0 (UP) to 90 (Down)
      //servoWrite(i_SgeeServoPin,i);
      servo.write(i);
      delay(10);
    }
  }
  servo.detach(); //Must detach servo pin after movement (sets pinMode to INPUT ending pwm events).
  Serial.println("Servo Detached");
} /*** END set_SgServo FN ***/

/*************************************************************************************************/
/*!
    \fn     set_Stop

    \brief  Set Motor control board L298N input pins all HIGH - Stops Motors

    \param  None

    \called Android -> Amdtps_main.c fn = amdtps_write_cback
                    -> BLE_example_funcs.cpp  fn = bleRxTxReceived sets amdtpsNano extern String s_Rcvd
                       with char of uint8_t byte array Android Command which is checked in
                       amdtpsNano.ino Loop function Switch statement if s_Rcvd has changed from 100.

    \Notes  Strange stuff ocurred if setting pwm to 0 so leave pwm alone and stop motors with high inputs

    \return None.
*/
/*************************************************************************************************/
void set_Stop( void ) { //also called when ble disconnected from amdtps_main.c amdtps_stop function
  s_Rcvd = "100"; //Set this to 100 to ignore switch statement in loop till new msg
  i_Moving = 0; //Stops the distance sensor calls.
  Serial.println("Stop Vinder Vaasher all Inp's high, Motors set 0");
  digitalWrite(i_Inp1, HIGH);
  digitalWrite(i_Inp2, HIGH);
  digitalWrite(i_Inp3, HIGH);
  digitalWrite(i_Inp4, HIGH);
  analogWrite(i_MotorPinLeft, 0);
  analogWrite(i_MotorPinRight, 0);
  //delay(100);
}

/*************************************************************************************************/
/*!
    \fn     set_Right

    \brief  Stop and turn slightly Right.

    \param  None

    \called Android -> Amdtps_main.c fn = amdtps_write_cback
                    -> BLE_example_funcs.cpp  fn = bleRxTxReceived sets amdtpsNano extern String s_Rcvd
                       with char of uint8_t byte array Android Command which is checked in
                       amdtpsNano.ino Loop function Switch statement if s_Rcvd has changed from 100.

    \Notes  Set Motor control board L298N input pins forward for motor1 and reverse for motor2
              Stops forward or reverse motion and turns a little bit based on time before setting all
              inputs to HIGH stopping both motors

    \return None.
*/
/*************************************************************************************************/
void set_Right( int i_Control ) { //1 = little 20 deg turn, 2 = turn little while moving, 3 = big 180 deg turn, 4 = 45 deg
  /*************************************************************************
       Stop and turn slightly... */

  s_Rcvd = "100"; //Set this to 100 to ignore switch statement in loop till new msg
  if (i_Control == 1 || i_Control == 3 || i_Control == 4) {
    i_Moving = 0; //Stops the distance sensor calls.
    analogWrite(i_MotorPinLeft, 0);
    analogWrite(i_MotorPinRight, 0);
    Serial.println("From start of set_Right - just set motor speeds to 0 in set_Right");
    delay(10);
    //Serial.print("from set_Forward i_Speed_Right should be 25, actually is = ");
    //Serial.println(i_Speed_Right);
    Serial.println("TURNING Right TURNING Right TURNING Right TURNING Right TURNING Right TURNING Right TURNING Right TURNING Right");
    digitalWrite(i_Inp1, HIGH);
    digitalWrite(i_Inp2, LOW);
    digitalWrite(i_Inp3, LOW);
    digitalWrite(i_Inp4, HIGH);
    delay(10);
    analogWrite(i_MotorPinLeft, 50000);
    analogWrite(i_MotorPinRight, 50000);
    if (i_Control == 1) delay(200);
    if (i_Control == 4) delay(600);
    if (i_Control == 3) delay(1200);
    // Stop motors - all 4 Inp's HIGH
    digitalWrite(i_Inp2, HIGH);
    digitalWrite(i_Inp3, HIGH);
    analogWrite(i_MotorPinLeft, 0);
    analogWrite(i_MotorPinRight, 0);
    Serial.println("From set_Right - just set all Inp's high, both motors to 0");
  }// End of if(i_Control == 1){
  if ( i_Control == 2) { //Righter
    i_Speed_Right = i_Speed_Right + 1000;
    analogWrite(i_MotorPinLeft, i_Speed_Left);
    analogWrite(i_MotorPinRight, i_Speed_Right);
  }
}

/*************************************************************************************************/
/*!
    \fn     set_Left

    \brief  Stop and turn slightly Left.

    \param  None

    \called Android -> Amdtps_main.c fn = amdtps_write_cback
                    -> BLE_example_funcs.cpp  fn = bleRxTxReceived sets amdtpsNano extern String s_Rcvd
                       with char of uint8_t byte array Android Command which is checked in
                       amdtpsNano.ino Loop function Switch statement if s_Rcvd has changed from 100.

    \Notes  Set Motor control board L298N input pins reverse for motor1 and forward for motor2.
              Stops forward or reverse motion and turns a little bit based on time before setting all
              inputs to HIGH stopping both motors

    \return None.
*/
/*************************************************************************************************/
void set_Left( int i_Control ) { //1 = little 20 deg turn, 2 = turn little while moving, 3 = big 180 deg turn, 4 = 45 deg
  /*************************************************************************
       Stop and turn slightly...*/
  s_Rcvd = "100"; //Set this to 100 to ignore switch statement in loop till new msg
  if (i_Control == 1 || i_Control == 3 || i_Control == 4) {
    i_Moving = 0; //Stops the distance sensor calls.
    analogWrite(i_MotorPinLeft, 0);
    analogWrite(i_MotorPinRight, 0);
    Serial.println("From start of set_Left - just set motor speeds to 0 in set_Left");
    delay(10);
    //Serial.print("from set_Forward i_Speed_Left should be 25, actually is = ");
    //Serial.println(i_Speed_Left);
    Serial.println("TURNING LEFT TURNING LEFT TURNING LEFT TURNING LEFT TURNING LEFT TURNING LEFT TURNING LEFT TURNING LEFT");
    digitalWrite(i_Inp1, LOW);
    digitalWrite(i_Inp2, HIGH);
    digitalWrite(i_Inp3, HIGH);
    digitalWrite(i_Inp4, LOW);
    delay(10);
    analogWrite(i_MotorPinLeft, 50000);
    analogWrite(i_MotorPinRight, 50000);
    if (i_Control == 1) delay(200); //150 = like 10 degrees
    if (i_Control == 4) delay(600); //like 180 deg
    if (i_Control == 3) delay(1200);
    // Stop motors - all 4 Inp's HIGH
    digitalWrite(i_Inp1, HIGH);
    digitalWrite(i_Inp4, HIGH);
    analogWrite(i_MotorPinLeft, 0);
    analogWrite(i_MotorPinRight, 0);
    Serial.println("From set_Left - just set all Inp's high, both motors to 0");
  } //End if i_Control == 1
  if ( i_Control == 2) { //Lefter
    i_Speed_Left = i_Speed_Left + 1000;
    analogWrite(i_MotorPinRight, i_Speed_Right);
    analogWrite(i_MotorPinLeft, i_Speed_Left);
  }
}

void set_Faster( void ) {
  s_Rcvd = "100"; //Disables case statement check in Loop Function till new message arrives
  i_Speed_Left = i_Speed_Left + 500;
  i_Speed_Right = i_Speed_Right + 500;
  analogWrite(i_MotorPinLeft, i_Speed_Left);
  analogWrite(i_MotorPinRight, i_Speed_Right);

}

void set_Slower( void ) {
  s_Rcvd = "100"; //Disables case statement check in Loop Function till new message arrives
  i_Speed_Left = i_Speed_Left - 500;
  i_Speed_Right = i_Speed_Right - 500;
  analogWrite(i_MotorPinLeft, i_Speed_Left);
  analogWrite(i_MotorPinRight, i_Speed_Right);

}

void scrub_Right() {
  set_Pump(1);
  set_Right(1); //right
  delay(200);
  set_ScrubMotor(60000); 
  delay(2000);
  set_ScrubMotor(0);
  set_Pump(0);
  delay(200);
  am_hal_wdt_restart();
}

void scrub_Left() {
  set_Pump(1);
  set_Left(1); //left
  delay(200);
  set_ScrubMotor(60000); 
  delay(2000);
  set_ScrubMotor(0);
  set_Pump(0);
  delay(200);
  am_hal_wdt_restart();
}

void step_Forward(int i_numCounts) {
  //Get speed from last foward movement..assumes will move forward with that pwm setting
  //Set both motors so they turn in the forward direction (depends on wiring)
  // Imp1 & 2 are the left motor, 3 & 4 are the right
  digitalWrite(i_Inp1, LOW);
  digitalWrite(i_Inp2, HIGH);
  digitalWrite(i_Inp3, LOW);
  digitalWrite(i_Inp4, HIGH);
  Serial.println("set_Forward - Just set Inp's H,L,H,L");
  delay(10);
  if (i_Speed_Right_Moving < 63000) {
    analogWrite(i_MotorPinLeft, i_Speed_Left_Moving + 2000); //Set as final number in set_Forward
    analogWrite(i_MotorPinRight, i_Speed_Right_Moving + 2000);
  }else {
    analogWrite(i_MotorPinLeft, i_Speed_Left_Moving); //Set as final number in set_Forward
    analogWrite(i_MotorPinRight, i_Speed_Right_Moving);    
  }
  delay(10);
  //Call check_SpeedRight to get the number of isr counts.  It returns the number of counts for 250 Msecs
  // each time it is called.
  int i_StepFrwdCounts = check_SpeedRight(); //250 MSecs
  while ( i_StepFrwdCounts > 0 && i_StepFrwdCounts < i_numCounts ) {
    am_hal_wdt_restart();
    i_StepFrwdCounts = i_StepFrwdCounts + check_SpeedRight(); //now 500 MSecs ...
    Serial.printf("Total counds so far %u", i_StepFrwdCounts);
  }
  set_Stop();

//  TRYING TO SEND NUMBER COUNTS BACK TO PHONE.....KHE TODO
//  FOR NOW JUST PUT BOT ON BLOCK AND SERIAL.PRINTLN
  //i_StepFrwdCounts = 1000;
  Serial.print("Final Number of Encoder Counts = ====================================================================== ");
  Serial.println(i_StepFrwdCounts);
/*  
  byte high = highByte(i_StepFrwdCounts);
  byte low = lowByte(i_StepFrwdCounts);
  Serial.printf("High = %u Low = %u\n",high,low);
  //byte val[2] = { high, low };
  //amdtpsSendData(val,2);
  //OK done moving that far now move left and right a couple of times - 2 sec scrub between moves
  scrub_Right();
  scrub_Right();
  scrub_Right(); //far right
  scrub_Left();
  scrub_Left();
  scrub_Left(); //Center
  scrub_Left();
  scrub_Left();
  scrub_Left(); //far left
  scrub_Right();
  scrub_Right();
  scrub_Right();
*/  
  am_hal_wdt_restart();
}

/*************************************************************************************************/
/*!
    \fn     set_Forward

    \brief  Move forward in a straight line

    \param  None

    \called Android -> Amdtps_main.c fn = amdtps_write_cback
                    -> BLE_example_funcs.cpp  fn = bleRxTxReceived sets amdtpsNano extern String s_Rcvd
                       with char of uint8_t byte array Android Command which is checked in
                       amdtpsNano.ino Loop function Switch statement if s_Rcvd has changed from 100.

    \Notes  Set Motor control board L298N input pins forward for motor1 and forward for motor2.
              Speed control has not been implemented yet

    \return None.
*/
/*************************************************************************************************/
void set_Forward( void ) {
  Serial.println("\n\n\n");
  s_Rcvd = "100"; //Disables case statement check in Loop Function till new message arrives
  i_currSpeedRight = 0;
  i_currSpeedLeft = 0;
  i_Speed_Left_Moving = 0;
  i_Speed_Right_Moving = 0;

  //Stop the wdt during this operation as it may exceed the timeout and cause a reboot
  //am_hal_wdt_halt();  //tried this but had failure with WDT stopped = get ladder and climb on roof to reset bot

  //i_Moving = 1; //Turns on the Distance check for the HC-SR04 sensor(s)
  i_Speed_Left = i_StartSpeedLeft; //never changes ie 6000
  i_Speed_Right = i_StartSpeedRight; //never changes ie 7000 = slight turn if same as left.
  set_Stop();
  //Serial.println("From start of set_Forward - just set_Stop return");
  delay(10);
  //Set both motors so they turn in the forward direction (depends on wiring)
  // Imp1 & 2 are the left motor, 3 & 4 are the right
  digitalWrite(i_Inp1, LOW);
  digitalWrite(i_Inp2, HIGH);
  digitalWrite(i_Inp3, LOW);
  digitalWrite(i_Inp4, HIGH);
  Serial.println("set_Forward - Just set Inp's H,L,H,L");
  delay(10);
  i_currSpeedRight = check_SpeedRight();
  i_currSpeedLeft = check_SpeedLeft();
  Serial.println("From set_Forward return From check_SpeedRight - B4 while increase should be 0 0 (Tics/250 ms) = ");
  Serial.print("i_currSpeedRight ");
  Serial.print(i_currSpeedRight);
  Serial.print(" i_currSpeedLeft ");
  Serial.print(i_currSpeedLeft);
  Serial.print(" i_CountEncRight ");
  Serial.print(i_CountEncRight);
  Serial.print(" i_CountEncLeft ");
  Serial.println(i_CountEncLeft);
  while ((i_currSpeedRight == 0 || i_currSpeedLeft == 0) && i_Speed_Left < 63000) { //Left is the higher one for this bot
    i_Speed_Left = i_Speed_Left + 2000;
    i_Speed_Right = i_Speed_Right + 2000;
    Serial.print("from set_Forward - While Loop PWM value now setting to : ");
    Serial.println(i_Speed_Left);
    am_hal_wdt_restart();
    //Serial.println("RESET WDT");
    analogWrite(i_MotorPinLeft, i_Speed_Left);
    analogWrite(i_MotorPinRight, i_Speed_Right);
    delay(10);
    i_currSpeedRight = check_SpeedRight();
    i_currSpeedLeft = check_SpeedLeft();
    Serial.print("From set_Forward in while loopreturn From check_SpeedRight - Should be 0 Current Speed (Tics/250 ms) = Right Left  ");
    Serial.print(i_currSpeedRight);
    Serial.print(" ");
    Serial.println(i_CountEncLeft);
  }
  if (i_Speed_Left < 63000) { // Add a little extra to give it a good start but don't exceed max of 65535 for 16 bit resolution
    analogWrite(i_MotorPinLeft, i_Speed_Left + 2000);
    analogWrite(i_MotorPinRight, i_Speed_Right + 2000);
    i_currSpeedRight = check_SpeedRight();
    i_currSpeedLeft = check_SpeedLeft();
    Serial.print("From set_Forward in while loopreturn From check_SpeedRight - Should be 0 Current Speed (Tics/250 ms) = Right Left  ");
    Serial.print(i_currSpeedRight);
    Serial.print(" ");
    Serial.println(i_CountEncLeft);
    Serial.print("End of set_Forward - Final PWM value now set to : ");
    Serial.println(i_Speed_Left + 2000);
    delay(10);
  }
    i_Speed_Left_Moving = i_Speed_Left;
    i_Speed_Right_Moving = i_Speed_Right;
    am_hal_wdt_restart();
  //Restart the WatchDogTimer (WDT)
  //am_hal_wdt_start();
}

/*************************************************************************************************/
/*!
    \fn     set_Reverse

    \brief  Move reverse in a straight line

    \param  None

    \called Android -> Amdtps_main.c fn = amdtps_write_cback
                    -> BLE_example_funcs.cpp  fn = bleRxTxReceived sets amdtpsNano extern String s_Rcvd
                       with char of uint8_t byte array Android Command which is checked in
                       amdtpsNano.ino Loop function Switch statement if s_Rcvd has changed from 100.

    \Notes  Set Motor control board L298N input pins reverse for motor1 and reverse for motor2.
              Speed control has not been implemented yet

    \return None.
*/
/*************************************************************************************************/
void set_Reverse( void ) {
  Serial.println("\n\n\n");
  s_Rcvd = "100"; //Disables case statement check in Loop Function till new message arrives
  i_currSpeedRight = 0;
  i_currSpeedLeft = 0;

  //Stop the wdt during this operation as it may exceed the timeout and cause a reboot
  am_hal_wdt_halt();

  //i_Moving = 1; //Turns on the Distance check for the HC-SR04 sensor(s)
  //am_hal_ctimer_clear(6, AM_HAL_CTIMER_TIMERA); //clear timer for pad 39 = pin 9
  //am_hal_gpio_state_write(39,AM_HAL_GPIO_OUTPUT_CLEAR);
  //am_hal_gpio_state_write(35,AM_HAL_GPIO_OUTPUT_CLEAR);
  i_Speed_Left = i_StartSpeedLeft;
  i_Speed_Right = i_StartSpeedRight;
  analogWrite(i_MotorPinLeft, 0);
  analogWrite(i_MotorPinRight, 0);
  Serial.println("From start of set_Reverse - just set motor speeds to 0 in set_Forward");
  delay(10);
  Serial.print("from set_Reverse i_Speed_Left should be 25, actually is = ");
  Serial.println(i_Speed_Left);
  //Set both motors so they turn in the forward direction (depends on wiring)
  // Imp1 & 2 are the left motor, 3 & 4 are the right
  digitalWrite(i_Inp1, HIGH);
  digitalWrite(i_Inp2, LOW);
  digitalWrite(i_Inp3, HIGH);
  digitalWrite(i_Inp4, LOW);
  delay(10);
  i_currSpeedRight = check_SpeedRight();
  i_currSpeedLeft = check_SpeedLeft();
  Serial.print("From set_Reverse return From check_SpeedRight - Should be 0 Current Speed (Tics/250 ms) = ");
  Serial.print(i_currSpeedRight);
  Serial.print(" ");
  Serial.print(i_currSpeedLeft);
  Serial.print(" ");
  Serial.println(i_CountEncRight);
  Serial.print(" ");
  Serial.println(i_CountEncLeft);
  while (i_currSpeedRight == 0 && i_Speed_Left < 64000) {
    i_Speed_Left = i_Speed_Left + 1000;
    i_Speed_Right = i_Speed_Right + 1000;
    Serial.print("from set_Reverse - While Loop PWM value now setting to : ");
    Serial.println(i_Speed_Left);
    analogWrite(i_MotorPinLeft, i_Speed_Left);
    analogWrite(i_MotorPinRight, i_Speed_Right);
    delay(10);
    i_currSpeedRight = check_SpeedRight();
    Serial.print("From set_Reverse in while loopreturn From check_SpeedRight - Should be 0 Current Speed (Tics/250 ms) = ");
    Serial.print(i_currSpeedRight);
    Serial.print(" ");
    Serial.println(i_CountEncRight);
  }
  if (i_Speed_Left < 63000) { //Don't exceed max of 65535 for 16 bit resolution
    analogWrite(i_MotorPinLeft, i_Speed_Left + 2000);
    analogWrite(i_MotorPinRight, i_Speed_Right + 2000);
    Serial.print("End of set_Reverse - Final PWM value now set to : ");
    Serial.println(i_Speed_Left + 500);
    delay(10);
  }
  //Restart the WatchDogTimer (WDT)
  am_hal_wdt_start();

}

/*************************************************************************************************/
/*!
    \fn     check_Distance

    \brief  Use HC-SR04 ultraSonic sensor to check distance to surface / wall and stop motors b4 collision
              or falling off roof.  Also sends measurement to Android (phone) for display.

    \param  None

    \called amdtpsNano.ino Loop function when i_Moving = 1 based on incrementing variable in loop which
                equals about 1/sec as set now.

    \msg    Sends distance measured (cm's) -> amdtps_main.c amdtpsSendData fn -> Android (Phone).

    \Notes  Had some trouble converting floats to char so decided to use int's as don't need precision to
              the decimal point for cm distances.  Did a lot of troubleshooting on this b4 arriving at this
              solution.

    \return None.
*/
/*************************************************************************************************/
void check_Distance(void) {
  char cbuf[5]; //Char array to hold the converted string from int to string conversion
  uint16_t len = 5; //expected type for the amdtpsSendData function that sends the data from amdtps_main.c //KHE ori = 6
  // which we changed from static void to extern void and added to amdtp_api.h as extern statement
  digitalWrite(trigPin, LOW); //Normal HC-SR04 stuff
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  //KHE 3rd param pulseIn 5000 = max distnace ` 80 - ceiling  = 0, 50000 ceiling = 40,
  duration7 = pulseIn(echoPin7, HIGH, 1000000); //3rd parameter is timeout microsec's before return 0 due to no pulse received ori = 50000
  distance7 = (duration7 * .0343) / 2;        // still takes a whole second to complete.
  if (distance7 < i_stopDistance || distance7 == 0) { //0 is a bad read...lose cable or something - should never happen
    //   but would like to stop if it does happen.
    //set_Stop();  //KHE REMOVE TO STOP ON DISTANCE = SOMETHING
  }
  String s_distance7 = (String)distance7; //Convert integer to string
  s_distance7.toCharArray(cbuf, 5); //Convert string to CharArray of size 5
  //Ble communicates with uint8_t arrays = byte arrays. So..we converted int->string->CharArray and sent the
  //  distance back to the phone with amdtpsSendData.
  amdtpsSendData((uint8_t*)cbuf, len);
}

void set_Pump(int i_PumpOnOff) {
  if (i_PumpOnOff == 1) {
    digitalWrite(i_Pump_Pin, HIGH);
  } else
  {
    digitalWrite(i_Pump_Pin, LOW);
  }
}


void loop() {
    l_Timer++;
    
    if (l_Timer > i_B4Timer2_LooperCount)  //80000
    {  //KHE 50000 about as fast as you might want
      l_Count++;
      //Serial.printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n LOOP TIMES UP......LOOP TIMES UP.......LOOP TIMES UP.......LOOP TIMES UP........LOOP TIMES UP......%d\n",l_Count);
//      delay(100);
      l_Timer = 0;
    }
  //Serial.println("Loop...."); //KHE Loops constantly....no delays
  //analogWrite(16,0); does stop it from moving around but kills all commands to ithe servo
  if (s_Rcvd != "100") //Check if we have a new message from amdtps_main.c through BLE_example_funcs.cpp
  {
    Serial.print("Received Msg - s_Rcvd = ");
    Serial.println(s_Rcvd);
    switch (s_Rcvd.toInt()) {
      case 0:
        set_Stop();
        break;
      case 1:
        set_Forward();
        break;
      case 2:
        set_Right(1);
        break;
      case 3:
        set_Reverse();
        break;
      case 4:
        set_Left(1); //1 = stop and turn
        break;
      case 5:
        set_ScrubMotor(40000);  //On
        break;
      case 6:
        set_ScrubMotor(0); //Off PWM value
        break;
      case 7:
        set_SgServo(0); // KHE TODO up - need to decide what value to set for servo
        break;
      case 8:
        set_SgServo(90); //down
        break;
      case 9:
        set_Left(2); //Lefter...trend left
        break;
      case 10:
        set_Right(2);
        break;
      case 11:
        set_Faster();
        break;
      case 12:
        set_Slower();
        break;
      case 13:
        set_Pump(1);
        break;
      case 14:
        set_Pump(0);
        break;
      case 15:
        step_Forward(20);
        break;
      case 16:
        //Big Left
        set_Left(3);
        break;
      case 17:
        //Big Right
        set_Right(3);
        break;
      case 67: //Decimal value of C for Connect
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("Turned Light ON");
        s_Rcvd = 100;  //KHE TODO CHANGE ME
        break;
      case 68: //Decimal value of D for Disconnect
        //Serial.println("got disconnect from case in ino file - set_Stop");
        i_Moving = 0; //stop measuring distance
        set_Stop(); //stop motors
        digitalWrite(LED_BUILTIN, LOW);
        //amdtps_conn_close();
        DmDevReset();
        //delay(250);
        //Serial.end();
        //setup();
        s_Rcvd = 100;
        break;
      default: 
        break;

    } //End switch cmd
    //Serial.print("Loop received new msg = ");
    //Serial.println(s_Rcvd);

  } //End if s_Rcvd != 100

  if (i_Moving == 1) { // Only check distance if moving forward
    l_Timer++;
    if (l_Timer > 80000) {  //KHE 50000 about as fast as you might want
      l_Timer = 0;
      check_Distance();
      //check_SpeedLeft();
      //check_SpeedRight();
    }
  }
  trigger_timers();

    // Disable interrupts.
  am_hal_interrupt_master_disable();




  //
  // Check to see if the WSF routines are ready to go to sleep.
  //
  if ( wsfOsReadyToSleep() )
  {
      am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
  }
  // Loop stops here on sleep and wakes on Timer2 interrupt, runs about 30 loops, then sleeps again.
  // An interrupt woke us up so now enable them and take it.
  am_hal_interrupt_master_enable();
  //am_watchdog_isr(); //just testing to make sure isr runs..it does but not firing
  delay(10);
} //END LOOP

void setupWdt() {
  Serial.println("############## setting up WatchDogTimer WDT ########################");
  Serial.print("Interrupt Count = "); Serial.print(g_sWatchdogConfig.ui16InterruptCount ); Serial.println(" ticks");
  Serial.print("Reset Count = "); Serial.print(g_sWatchdogConfig.ui16ResetCount); Serial.println(" ticks");

  // (Note: See am_hal_reset.h for RESET status structure)
  am_hal_reset_status_t sStatus;

  // Print out reset status register. 
  // (Note: Watch Dog Timer reset = 0x40)
  am_hal_reset_status_get(&sStatus);
  resetStatus = sStatus.eStatus;

  char rStatus[3];
  sprintf(rStatus, "Reset Status Register = 0x%x\n", resetStatus);
  Serial.println(rStatus);

  // Set the clock frequency.
  am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

  // Set the default cache configuration
  am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
  am_hal_cachectrl_enable();

  // Configure the board for low power operation.
  am_bsp_low_power_init();

  // Clear reset status register for next time we reset.
  am_hal_reset_control(AM_HAL_RESET_CONTROL_STATUSCLEAR, 0);

  // LFRC must be turned on for this example as the watchdog only runs off of the LFRC.
  am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_LFRC_START, 0);

}

// *****************************************************************************
//
// Interrupt handler for the watchdog.
//
// *****************************************************************************
extern void am_watchdog_isr(void) {
  Serial.printf("\n\n\n\n\n@@@@@@@@@@@ GOT WATCHDOG TIMER - Counter = %d !!!!!!!!RE-BOOTING!!!!!!!!\n",watchdogCounter);

  // Clear the watchdog interrupt.
  am_hal_wdt_int_clear();

  // Catch the first four watchdog interrupts, but let the fifth through untouched.
  if ( watchdogCounter < 4 ) {
    // Restart the watchdog.
    //am_hal_wdt_restart(); // "Pet" the dog.
    if ( watchdogCounter == 3) {
       Serial.printf("\n\n@@@@@@@ RE-BOOTING @@@@@@@@@@@@@@\n");  
    }
  }
  else {
    digitalWrite(LED_BUILTIN, HIGH); // Indicator that a reset will occur. 
    //print statements won't be seen as we are already re-booting
  }

  // Increment the number of watchdog interrupts.
  watchdogCounter++;
}

extern void timer_isr(void) {
  count++;
  uint32_t ui32Status;
  Serial.printf("\n\n@@@@@@@@@@@@@@@@@@@@@@@@@@ Timer2 ISR @@@@@@@@@@@@@@@@@=========> Count = %d\n",count);
  //Serial.printf("@@@@ WATCHDOG TIMER COUNTER = %d\n",watchdogCounter);
  ui32Status = am_hal_ctimer_int_status_get(true);
  am_hal_ctimer_int_clear(ui32Status);
  if (count % 2 == 0) {
    digitalWrite(blinkPin, LOW);
  }
  else {
    digitalWrite(blinkPin, HIGH);
  }
  i_B4Timer2_LooperCount = 10; //Used this to count number of times loop loops every wakeup
  // Restart the watchdog.
  //am_hal_wdt_restart(); // Stop re-boot
  amdtpsSendData((uint8_t*)valEnq, 3); //Sending phone Hex for e n q = uint8_t valEnq [] = {(byte)0x65,(byte)0x6e,(byte)0x71};
  if ( count == 2) {
    am_hal_wdt_start();  
    Serial.printf("\n\n\n\n\n@@@@@@@@@@@@@@@@@@@@@ STARTING WATCHDOGTIMER WDT STARTING @@@@@@@@@@@@@@@@@@@\n\n\n\n\n");
  }
  // Phone is set up to respond 
}

/* 
 * This routine wl update the WSF timers on a regular base
 */

void trigger_timers() //Called from loop when the chip is awakened
{
  //
  // Calculate the elapsed time from our free-running timer, and update
  // the software timers in the WSF scheduler.
  //
  update_scheduler_timers(); //We've woken up so set/check timers in BLE_exampole_funcs.cpp
  wsfOsDispatcher();            // start any handlers if event is pending on them

  //
  // Enable an interrupt to wake us up next time we have a scheduled event.
  //
  set_next_wakeup();

}
