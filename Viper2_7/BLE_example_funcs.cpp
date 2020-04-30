#include "BLE_example.h"
extern String s_Rcvd; //KHE declared in .ino file
extern String s_AdvName; //KHE declared in .ino file
//KHEKHE
//extern const uint8_t amdtpScanDataDisc[22] = {8,DM_ADV_TYPE_LOCAL_NAME,'R','E','V','1','_','3','W'}; //KHE WORKS
//extern uint8_t amdtpScanDataDisc[22] = {}; //works but warns initialized and declared 'extern'

uint8_t amdtpScanDataDisc[22];
//*****************************************************************************
//
// Timer configuration macros.
//
//*****************************************************************************
#define MS_PER_TIMER_TICK           10  // Milliseconds per WSF tick
#define CLK_TICKS_PER_WSF_TICKS     5   // Number of CTIMER counts per WSF tick.

//*****************************************************************************
//
// WSF buffer pools.
//
//*****************************************************************************
#define WSF_BUF_POOLS               4

// Important note: the size of g_pui32BufMem should includes both overhead of internal
// buffer management structure, wsfBufPool_t (up to 16 bytes for each pool), and pool
// description (e.g. g_psPoolDescriptors below).

// Memory for the buffer pool
static uint32_t g_pui32BufMem[(WSF_BUF_POOLS*16
         + 16*8 + 32*4 + 64*6 + 280*8) / sizeof(uint32_t)];

// Default pool descriptor.
static wsfBufPoolDesc_t g_psPoolDescriptors[WSF_BUF_POOLS] =
{
    {  16,  8 },
    {  32,  4 },
    {  64,  6 },
    { 280,  8 }
};

//*****************************************************************************
//
// Tracking variable for the scheduler timer.
//
//*****************************************************************************
uint32_t g_ui32LastTime = 0;
//extern "C" void radio_timer_handler(void);

//*****************************************************************************
//
// Initialization for the ExactLE stack.
//
//*****************************************************************************
void exactle_stack_init(void){
    wsfHandlerId_t handlerId;

    //
    // Set up timers for the WSF scheduler.
    //
    scheduler_timer_init();
    WsfTimerInit();

    //
    // Initialize a buffer pool for WSF dynamic memory needs.
    //
    WsfBufInit(sizeof(g_pui32BufMem), (uint8_t*)g_pui32BufMem, WSF_BUF_POOLS, g_psPoolDescriptors);

    //
    // Initialize security.
    //
    SecInit();
    SecAesInit();
    SecCmacInit();
    SecEccInit();

    //
    // Set up callback functions for the various layers of the ExactLE stack.
    //
    handlerId = WsfOsSetNextHandler(HciHandler);
    HciHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(DmHandler);
    DmDevVsInit(0);
    DmAdvInit();
    DmConnInit();
    DmConnSlaveInit();
    DmSecInit();
    DmSecLescInit();
    DmPrivInit();
    DmHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(L2cSlaveHandler);
    L2cSlaveHandlerInit(handlerId);
    L2cInit();
    //L2cSlaveInit(); needed ???????????????????????????????//

    handlerId = WsfOsSetNextHandler(AttHandler);
    AttHandlerInit(handlerId);
    AttsInit();
    AttsIndInit();

    handlerId = WsfOsSetNextHandler(SmpHandler);
    SmpHandlerInit(handlerId);
    SmprInit();
    SmprScInit();
    HciSetMaxRxAclLen(251);

    handlerId = WsfOsSetNextHandler(AppHandler);
    AppHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(AmdtpHandler);
    AmdtpHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(HciDrvHandler);
    HciDrvHandlerInit(handlerId);
}



//*****************************************************************************
//
// Set up a pair of timers to handle the WirelessSoftwareFoundation (WSF) scheduler.
//
//*****************************************************************************
void
scheduler_timer_init(void)
{
  /**************************************************************************
     CONFIGURE TIMER2 REPEATING TIMER - RESETS WDT  
      - Interrupts and sends e n q HEX to phone which responds with ACK.  Ack is 
        processed as CNF message response where the WatchDogTimer (WDT) is 
        restarted.  If there is no ACK response from phone,the chip will not 
        get the WDT restart and will reboot, stopping the motors and stopping
        any attempt at flight from the roof.    

     Refer to Ambiq Micro Apollo3 Blue MCU datasheet section 13.2.2
     and am_hal_ctimer.c line 710 of 2210.
  ***************************************************************************/
  am_hal_ctimer_config_single(2, AM_HAL_CTIMER_TIMERA,
                              AM_HAL_CTIMER_LFRC_32HZ |
                              AM_HAL_CTIMER_INT_ENABLE |
                              AM_HAL_CTIMER_FN_REPEAT); 

  /*
    Set the timing parameters.
    32 Hz * 0.5 seconds = 16 ticks = 15 ticks off and 1 tick of high output

    Repeated Count: Periodic 1-clock-cycle wide pulses with optional interrupts.
    The last parameter to am_hal_ctimer_period_set() has no effect in this mode.
  */
  am_hal_ctimer_period_set(2, AM_HAL_CTIMER_TIMERA, 62, 1);

  /*
    Register an interrupt service routine that will be called later.
  */
  am_hal_ctimer_int_register(AM_HAL_CTIMER_INT_TIMERA2C0, timer_isr); // INTterrupt defined in am_hal_ctimer
  //timer_isr function is in .ino file as extern and forward declared in BLE_example.h
  /*
    Clear and enable the selected timer.
  */
  am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA2C0); // Clear Timer 2 interrupt
  am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA2C0); // Enable Timer 2 interrupt
  // *****************************************************************************
  //                        END OF TIMER2 REPEATING TIMER
  // *****************************************************************************


  //******************************************************************************
  //                        SYSTEM TIMERS
  // One of the timers will run in one-shot mode and provide interrupts for
  //  scheduled events such as BLE messages.
  //******************************************************************************
  am_hal_ctimer_clear(0, AM_HAL_CTIMER_TIMERA); //Clear Timer 0, segment TimerA
  am_hal_ctimer_config_single(0, AM_HAL_CTIMER_TIMERA,
                              (AM_HAL_CTIMER_INT_ENABLE | 
                                AM_HAL_CTIMER_LFRC_512HZ |
                                AM_HAL_CTIMER_FN_ONCE)); 
  //
  // The other timer will run continuously and provide a constant time-base.
  //
  am_hal_ctimer_clear(0, AM_HAL_CTIMER_TIMERB); //Clear Timer 0, segment TimerB 
  am_hal_ctimer_config_single(0, AM_HAL_CTIMER_TIMERB,
                                (AM_HAL_CTIMER_LFRC_512HZ |
                                AM_HAL_CTIMER_FN_CONTINUOUS));
  //
  // Start the continuous timer.
  //
  am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERB);
  //
  // Enable the timer interrupt.
  //
  am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0);
  NVIC_EnableIRQ(CTIMER_IRQn);
  //nested vector interrupt controller (NVIC) is Cortex-M design for 
  //  interrupt handling. 
}

//*****************************************************************************
//
// Calculate the elapsed time, and update the WSF software timers.
//
//*****************************************************************************
void
update_scheduler_timers(void)
{
    uint32_t ui32CurrentTime, ui32ElapsedTime;

    //
    // Read the continuous timer.
    //
    ui32CurrentTime = am_hal_ctimer_read(0, AM_HAL_CTIMER_TIMERB);

    //
    // Figure out how long it has been since the last time we've read the
    // continuous timer. We should be reading often enough that we'll never
    // have more than one overflow.
    //
    // Do not make the number of click, when ui32CurrentTime < g_ui32LastTime
    // a very high number. it will cause ALL the WSF - timers to time out !!
    // resulting in unexpected error messages
    //
    ui32ElapsedTime = (ui32CurrentTime >= g_ui32LastTime ?
                       (ui32CurrentTime - g_ui32LastTime) :
                       CLK_TICKS_PER_WSF_TICKS + 1 );   // pvh
                      // (0x10000 + ui32CurrentTime - g_ui32LastTime));

    //
    // Check to see if any WSF ticks need to happen.
    //
    if ( (ui32ElapsedTime / CLK_TICKS_PER_WSF_TICKS) > 0 )
    {
        //
        // Update the WSF timers and save the current time as our "last
        // update".
        //
        WsfTimerUpdate(ui32ElapsedTime / CLK_TICKS_PER_WSF_TICKS);

        g_ui32LastTime = ui32CurrentTime;
    }
}

//*****************************************************************************
//
// Set a timer interrupt for the next upcoming scheduler event.
//
//*****************************************************************************
void
set_next_wakeup(void)
{
    bool_t bTimerRunning;
    wsfTimerTicks_t xNextExpiration;

    //
    // Stop and clear the scheduling timer (one-shot).
    //
    am_hal_ctimer_stop(0, AM_HAL_CTIMER_TIMERA);
    am_hal_ctimer_clear(0, AM_HAL_CTIMER_TIMERA);

    //
    // Check to see when the next timer expiration should happen. bTimerRunning is 
    //  true if timer is running and WsfTimerNextExpiration returns number of 
    //  ticks left on timer.  When 0, timer is expired and would be serviced. 
    //
    xNextExpiration = WsfTimerNextExpiration(&bTimerRunning);

    //
    // If there's a pending WSF timer event, set an interrupt to wake us up in
    // time to service it. Otherwise, set an interrupt to wake us up in time to
    // prevent a double-overflow of our continuous timer.
    //
    if ( xNextExpiration )
    {
        am_hal_ctimer_period_set(0, AM_HAL_CTIMER_TIMERA,
                                 xNextExpiration * CLK_TICKS_PER_WSF_TICKS, 0);
    }
    else
    {
        am_hal_ctimer_period_set(0, AM_HAL_CTIMER_TIMERA, 0x8000, 0);//0x8000 = 32768
    }

    //
    // Start the one-shot scheduling timer.
    //
    am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);
}

//*****************************************************************************
//
// Interrupt handler for the CTIMERs
//
//*****************************************************************************
extern "C" void am_ctimer_isr(void){
    uint32_t ui32Status;
    //Serial.printf("\n\n\n\n\n@@@@@@@@@@@@@@@@ GOT Timer0 ISR  @@@@@@@@@@@@@@@@@@@@\n");
    //
    // Check and clear any active CTIMER interrupts.
    //
    ui32Status = am_hal_ctimer_int_status_get(true);
    am_hal_ctimer_int_clear(ui32Status);
    //Serial.printf("BLE_example_funcs.cpp am_ctimer_isr : ISR Event = %08lX\n",ui32Status);

    //
    // Run handlers for the various possible timer events.
    //
    am_hal_ctimer_int_service(ui32Status);
}

//*****************************************************************************
//
// Interrupt handler for BLE
//
//*****************************************************************************
extern "C" void am_ble_isr(void){
    HciDrvIntService();
}

// ****************************************
// 
// C-callable functions KHE
// 
// ****************************************
//extern "C" void UserRequestRec(uint8_t * buf, uint16_t len){
//  UserRequestReceived(buf, len);
//}

extern uint8_t bleRxTxReceived(uint8_t * buf, uint16_t len)
{
  int i_Count = 0;
  Serial.printf("BLE_example_funcs.cpp bleRxTxReceived buf = ");      
  while ( i_Count < len) {
    Serial.printf("%x",buf[i_Count]); 
    Serial.printf(" ");
    i_Count ++;   
  }
  Serial.printf("\r\n");
  s_Rcvd = (char*) buf; //KHE convert msg into string
  //Serial.print("s_Rcvd = $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
  //Serial.println(s_Rcvd);
  // PRINTING BYTE MESSAGE AS CHAR:
  //Serial.print("char* buf = ");
  //Serial.println((char *) buf);
  //debug_printf("Received Message: %s\n",buf); //easiest
  //Serial.printf("Serial.printf message received : %s\n",buf);
  //Serial.print("Received Message from BLE_example_funcs.cpp Serial.println((char)*buf); - bleRxTxReceived buf = : ");
  //Serial.println((char)*buf); //Works converts byte 49 to 1 and byte 50 to 2 = casting (pointer to byte) into char
  /*****************************************************************************************************************
   * KHE On connection..toggle state of BUILTIN LED .. connected = on **********************************************
   *  Could use this area to check for String messages and do something with those, tried to read value from pMsg,
   *    but it only received a 01 byte so interfered with Arduino messageing...oh well.
  *****************************************************************************************************************/
  //if(s_Rcvd = "[01]") {
  //  Serial.println("RECEIVED CONNECTON MSG");
  //  if(!digitalRead(LED_BUILTIN)){
  //    digitalWrite(LED_BUILTIN, HIGH);
  //  } else digitalWrite(LED_BUILTIN,LOW);

  //}
  
  return * buf;
}
extern void RobotConn (int i_Conn) {
  //Serial.println("*******************GOT DISCONNECT***********************");
  if ( i_Conn == 0){
    s_Rcvd = "68"; //D (Disconnect)
  }
  if ( i_Conn == 1){
    s_Rcvd = "67"; //C (Connect)
  }
  //Serial.print("s_Rcvd = ");
  //Serial.println(s_Rcvd);
  
}

extern void set_led_high( void ){
  digitalWrite(LED_BUILTIN, HIGH);
}

extern void set_led_low( void ){
  digitalWrite(LED_BUILTIN, LOW);
}

/*************************************************************************************************/
/*!
    \fn     set_Adv_Name

    \brief  Parses global ino file string into BLE advertising name in amdtpScanDataDisc structure

    \param  none

    \called Arduino setup function, first thing just sets name in structure for future use

    \return None.
*/
/*************************************************************************************************/
void set_Adv_Name(void){
  //following line works - came from amtdp_main.c close to beginning
  //extern const uint8_t amdtpScanDataDisc[22] = {8,DM_ADV_TYPE_LOCAL_NAME,'R','E','V','1','_','3','W'};
  
  int n = s_AdvName.length(); //Length of string set in ino file as global String (doesn't include teminator) 
  if ( n > 20) {
    s_AdvName = s_AdvName.substring(0,20); //Only allow up to 20 char's
    n = s_AdvName.length(); //need length to change to uint8_t (new length after substring)
  }
  int i = 2; //1st char(0) is adv type char + # of char's, 2nd char(1)  is adv type
  char c_AdvName[n+1]; //create Array large enough to hold chars of string + teminator char 
  strcpy(c_AdvName, s_AdvName.c_str()); //copy string to char array
  //1st char in array must be = to # char's in name (no terminator) + 1 for adv. type.  If off by even 1, 
  //  the name will not show up in scans
  amdtpScanDataDisc[0] = n+1; //set first array pos to # of char's + 1 for the type of adv packet
  amdtpScanDataDisc[1] = DM_ADV_TYPE_LOCAL_NAME; // type of adv packet
  for(i=2; i<n+2; i++){ //Add the char's from the string to the amdtpScanDataDisc array starting at pos 2
    amdtpScanDataDisc[i] = c_AdvName[i-2];
  }
}
// ****************************************
//
// Debug print functions
//
// ****************************************
#if (defined BLE_Debug) || (defined BLE_SHOW_DATA)

#define DEBUG_UART_BUF_LEN 256
char debug_buffer [DEBUG_UART_BUF_LEN];

extern "C" void debug_float (float f) {
  SERIAL_PORT.print(f);
}

extern "C" void debug_print(const char* f, const char* F, uint16_t L){
  SERIAL_PORT.printf("fm: %s, file: %s, line: %d\n", f, F, L);
}

extern "C" void debug_printf(char* fmt, ...){
    va_list args;
    va_start (args, fmt);
    vsnprintf(debug_buffer, DEBUG_UART_BUF_LEN, (const char*)fmt, args);
    va_end (args);

    SERIAL_PORT.print(debug_buffer);
}
#else  // do nothing
extern "C" void debug_printf(char* fmt, ...){}

#endif //BLE_Debug & BLE_SHOW_DATA

//*****************************************************************************
//
// Display debug messages messages.
//
//*****************************************************************************
#ifdef AM_DEBUG_PRINTF
extern "C" void uart_string_print(char *mess) {
  Serial.print(mess);
}

void enable_print_interface()
{
  am_util_stdio_printf_init(uart_string_print);
}
#endif
