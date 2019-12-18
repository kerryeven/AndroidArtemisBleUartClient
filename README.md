# AndroidArtemisBleUartClient
android client
# Contains:

# 1) Android Studio Project KERobotNrf
# 2) ble_freertos_amdtpc – Client ble modified from AmbiqMicro SDK example
# 3) ble_freertos_amdtps – Server ble modified from AmbiqMicro SDK example

Power Edge2 with server code, blue led should light  
Start Android KERobotNrf (KERobotNrf Edge2 Uart icon)  
- click connect button.  New dialog appears should show amdtp in scan results.  If not keep hitting scan until you see amdtp when scan becomes available.  Sometimes it takes several attempts.   Try re-setting server board if not seen.  
- click on text box at bottom.  Type 1 send and yellow led lights, type 2 send and red led lights.  Disconnect and led’s go back to blue.  Whatever you type will echo back to your screen as rx and that response is coming from the server board!  Simple two way communication over ble to artemis!


Embedded Debug Journey:  Getting Edge2 server to turn on/off lights with messages sent and returned over ble from Android.  Start server = blue light.  Send 1 = yellow. Send 2 = red.  Disconnect = blue.  

Segger Ozone v3/10:  Just a couple of pre-requisites....But quite easy and successful.  I ended up using this as it is very simple and reliable and not prone to the mis-configurations I kept running into with eclipse.  

Segger Embedded Studio for ARM:  Spent a couple of days trying to get the hang of this ide.  Althought it was easy to get a hello world app working and debugging, I was unable to successfully get the amdtp application to install or run.  

Eclipse: I also downloaded and installed eclipse with gnu arm and mcu (with segger j-link debug) add-ons The eclipse install took a good 12 hours or so to get running, but has an excellent debugger. 

Hardware:  J-Link edu mini.  I ordered from Sparkfun and it came with firmware version 1.0.  That version is not ugradeable and will cause problems if you click yes to the repetitively offered suggestion strongly advising you to update the firmware.  Don’t do that, but if you do, the Segger Configurator will allow you to restore the original firmware.  

- Header - 2x5 Pin (Male, 1.27mm)  SF Part # 15362 – Goes between board and cable to J-link.
- J-Link EDU Mini Programmer SF Part # 15345

Use rubber band to hold header/cable to board by wrapping them after attaching.  Works great.


Software: Follow Sparkfun Using SparkFun Edge Board with Ambiq Apollo3 SDK tutorial section: Toolchain Setup.  You should end up with a directory from wherever you installed the Ambiq SDK/boards_sfe/edge2/examples.   Copy the SDK/boards/apollo3_evb/ble_freertos_amdtps to your edge2/examples directory.  You should be able to change directory in bash shell terminal to the gcc directory of the newly copied files.  Type make and it should compile without errors creating a bin file etc.   Easiest option once you have make working from the SDK files you copied would be to delete the directories you just created and download this example from github.  

Download Segger J-Link Software and Documentation Pack and Ozone and install using defaults.  

Code examples:  Follow the SparkFun tutorial for the edge Board on installing gcc toolchain.  They also have a note on developing Atemis and Arduino that walks through installing the Ambiq SDK and the SparkFun mod’s for working with Ambiq’s examples.  The Sparkfun files are all installed to their own directory structure under boards_sfe.    

	I compiled all code with gcc make via Git Bash shell (SparkFun app note install) using SF’s model Makefile found within the boards_sfe directory.  Simply add SF’s unique sections and headers to the Makefiles that are included in the gcc directory of the examples but use the SF Rules to get the bootloader’s to work.    Use the example’s includes, src’s, etc.etc.  Just keep doing make and make clean, ‘till you clear up any errors.  It’s tricky but it will work so keep trying.   Once you have a clean make, you can flash via usb with make bootload_asb.  Once you have working makes, you can also debug/flash with ozone.

Note:  Flashing with make bootload_asb will corrupt the normal Arduino bootloader but is easily fixed via Arduino Tools/Svl Baud Rate = 115200 then Tools/Burn Bootloader.  That will load the SparkFun svl bootloader back on an you’ll be good to resume using normal Android on the Artemis boards. 

J-Link Probe:  Hook up J-Link to usb port.  Attach cable with header pins to board.  Pin one is on the side of the cable with the raised middle to the right.  It needs to go into the whole marked with a | next to it on one side of the board.   Edge2 board needs to be attached from the back but it only will fit one way on the boards.  Wrap the rubber band around the board with the cable pin header attached to the board to hold the pins to the board at a slight angle.  It will make good contact that way and stay in place.  

Plug usb/J-link/board into usb and start J-Link Commander program.   If J-link is good, you’ll see a terminal window with ...ok, version, serial#, etc.  The green light on the J-Link will be pretty much solid with a little flickering.  Connect your board to another usb 5v power source.  Type connect into Commander.  Type ? for device and a window will pop up.  Scroll to AmbiqMicro and select AMA3B1KK-KBR.  (Sparkfun boards as of 12/19).  Choose SWD (not JTAG).  Lots of info and you are good to go for debugging. 

Ozone:  Install with defaults.  This is incredibly easy compared to Eclipse and SES.  Open Ozone.  Click the Create New Project with the wand on it (Wizard).  Select device (same sort of panel).  Leave peripherals blank.  Click Next.  SWD, 1MHZ, USB.  Click Next. Click the ... button to browse to your GCC directory where you created your Makefile/bin and find the .axf file.  Select that .axf file.    That’s it, your ready to flash and or debug.   You can attach to a running board without flashing if you prefer.  

If you get an error regarding freeRTOS , you are using that in your code.  Ignore the message.  Exit Ozone and save the changes to a new project name of your choice.  You need to open your project file which will by in your user directory Users/You with an extension of .jdebug in Notepad++ and insert the line:

Project.SetOSPlugin("freertosPlugin");

Into the OnProjectLoad section of  the file as shown.  Then save the project file, restart Ozone, and debug away.  Wow, that was easy!  
/*********************************************************************  
//  
//       OnProjectLoad  
//  
* Function description  
*   Project load routine. Required.  
//  
**********************************************************************  
*/  
void OnProjectLoad (void) {  
  //  
  // Dialog-generated settings  
  //  
  Project.AddPathSubstitute ("C:/Users/kerry", "$(ProjectDir)");  
  Project.AddPathSubstitute ("c:/users/kerry", "$(ProjectDir)");  
  Project.SetDevice ("AMA3B1KK-KBR");  
  Project.SetHostIF ("USB", "");  
  Project.SetTargetIF ("SWD");  
  Project.SetTIFSpeed ("1 MHz");  
  Project.AddSvdFile ("C:/Program Files/SEGGER/Ozone V3.10/Config/CPU/Cortex-M4F.svd");  
  //  
  // User settings  
  //  
  Project.SetOSPlugin("freertosPlugin");  
  File.Open ("$(ProjectDir)/Documents/AmbiqSuite-Rel2.2.0/boards_sfe/edge2/examples/ble_freertos_amdtps/gcc/bin/ble_freertos_amdtps_asb.axf");  
}  

- Setup with J-Link in one usb and power to board through another power supply and you can run a terminal program like Termite at the same time that you debug in Ozone.  That allows you to control the client from the terminal and see/debug the messages etc., from Ozone.  

AMDTPC example: (Used Sparkfun Redboard Nano board)

radio_task.c comment out if ( wsfOsReadyToSleep() ) statement to stop sleeping and waking and the associated messages.

getting printf statements out to the pc screen was accomplished by adding am_menu_printf (“text you want”) anywhere after the ble_menu definition at the end of ble_freertos_amdtpc.c file.  

DEBUG NOTE: - How to print array item, return, and line feed
data[1] = counter;
    status = AmdtpcSendPacket(AMDTP_PKT_TYPE_DATA, false, true, data, sizeof(data));
    am_menu_printf("SendPacketReturn = %d",status); //KHE 
    am_menu_printf(", Data Sent = %d\n",data[1]);
    am_menu_printf("amdtp_main - just sent data\n");  //KHE  actually only sends once

Can’t am_menu_printf in amdtpc_main.c.  Had to add:
	extern uint32_t am_menu_printf(const char *pcFmt, ...);
	after the include section at the top of the fle.   

amdtpServiceUUID 		00002760-08c2-11e1-9073-0e8ac72e1011
amdtpRxChUuid: 		00002760-08c2-11e1-9073-0e8ac72e0011
amdtpTxChUuid:		00002760-08c2-11e1-9073-0e8ac72e0012
attCliChCfgUuid: amdtpTxCcc 	181c2800-2801-2802-2803-290029012902
amdtpAckChUuid: 		00002760-08c2-11e1-9073-0e8ac72e0013
attCliChCfgUuid: amdtpAckCcc	181c2800-2801-2802-2803-290029012902


radio_task.c comment out if ( wsfOsReadyToSleep() ) statement to stop sleeping and waking and the associated messages.


 Turn the blue light on at power up...

	Add to ble_freertos_amdtps.c near end after #ifdef AM_DEBUG_PRINTF

am_hal_gpio_pinconfig(18, g_AM_HAL_GPIO_OUTPUT);
    	am_hal_gpio_output_set(18);

 amdtpc client example (second level) - Turn on yellow light when client sends 3, (red 4)  from serial 
  
Add pinconfig’s for other lights to ble_freertos_amdtps.c:
    am_hal_gpio_pinconfig(17, g_AM_HAL_GPIO_OUTPUT); //KHE Green
    am_hal_gpio_pinconfig(18, g_AM_HAL_GPIO_OUTPUT); //KHE Blue
    am_hal_gpio_pinconfig(19, g_AM_HAL_GPIO_OUTPUT); //KHE Red
    am_hal_gpio_pinconfig(37, g_AM_HAL_GPIO_OUTPUT); //KHE Yellow 

Add to amdtp_main.c: amdtpDtpRecvCback function if buf[0] == 1
 	  if (buf[0] == 1)  
    {  
        APP_TRACE_INFO0("send test data\n");  
        am_hal_gpio_output_clear(19);  //KHE  
        am_hal_gpio_output_set(37);  //KHE  
    //    AmdtpsSendTestData(); //KHE  
    }  
    else if (buf[0] == 2)  
    {  
        APP_TRACE_INFO0("send test data stop\n");  
        am_hal_gpio_output_set(19);  
        am_hal_gpio_output_clear(37);  
        sendDataContinuously = false;  
    }  
Turns out if not in amdtpDtpRecvCback, can’t turn lights on from amdtpc nano client.  I was thinking that if I put it in following amdtps_main.c it allowed all functionality to work.  But no, needs this to light the lights from the nano client.
 
 Nrf connect to turn lights on....

- Amdtps_main.c amtps_write_cback 
- Add the following begore return AMDTP_SUCCESS

    if (pValue[0] == 1)  
    {  
        //KHE  
        am_hal_gpio_output_clear(19);  //KHE  
        am_hal_gpio_output_set(37);  //KHE  
    }  
    if (pValue[0] == 2)  
    {  
        am_hal_gpio_output_set(19);  
        am_hal_gpio_output_clear(37);  
    }  

		
 Change pValue above in amtps_write_cback to 49 and 50, the decunal value of the utf-8 numbers entered in the android app.  The app converts it to binary for transmission.  Android 


Working....not showing up in scan routinely...have to hit many times sometimes.




