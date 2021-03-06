#ifndef _BLE_EXAMPLE_H_
#define _BLE_EXAMPLE_H_

#include "Arduino.h"
#include "ble_debug.h"

//#define DEBUG
#define SERIAL_PORT Serial

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include "wsf_types.h"
#include "wsf_trace.h"
#include "wsf_buf.h"

#include "hci_handler.h"
#include "dm_handler.h"
//#include "dm_api.h"
#include "l2c_handler.h"
#include "att_handler.h"
#include "smp_handler.h"
#include "l2c_api.h"
#include "att_api.h"
#include "smp_api.h"
#include "app_api.h"
#include "hci_core.h"
#include "hci_drv.h"
#include "hci_drv_apollo.h"
#include "hci_drv_apollo3.h"

#include "am_mcu_apollo.h"
#include "am_util.h"

#include "amdtp_api.h"
#include "app_ui.h"

#include "wsf_msg.h"
#include "am_bsp.h"

#ifdef __cplusplus
}
#endif

// command to sent between client and server
// make sure to stay in sync with the list on the client !!!
typedef enum eAmdtpcmd
{
    AMDTP_CMD_NONE,
    AMDTP_CMD_START_TEST_DATA,
    AMDTP_CMD_STOP_TEST_DATA,
    AMDTP_CMD_HELLO,
    AMDTP_CMD_REQ_BATTERY_LEVEL,
    AMDTP_CMD_REQ_BATTERYLOAD_ON,
    AMDTP_CMD_REQ_BATTERYLOAD_OFF,
    AMDTP_CMD_REQ_INTERNAL_TEMP_CEL,
    AMDTP_CMD_REQ_INTERNAL_TEMP_FRH,
    AMDTP_CMD_TURN_LED_ON,
    AMDTP_CMD_TURN_LED_OFF,
    AMDTP_CMD_BME280,
    AMDTP_CMD_ADC,
    AMDTP_CMD_CHAT,
    AMDTP_CMD_READ_PIN,
    AMDTP_CMD_PIN_HIGH,
    AMDTP_CMD_PIN_LOW,
    AMDTP_CMD_CUSTOM1,
    AMDTP_CMD_CUSTOM2,
    AMDTP_CMD_CUSTOM3,
    AMDTP_CMD_CUSTOM4,
    AMDTP_CMD_CUSTOM5,
    AMDTP_CMD_MAX
}eAmdtpPktcmd_t;

// ****************************************************************************
//
//  Default coefficients (used when calibratied trims not provided):
//  TEMP_DEFAULT    = Temperature in deg K (e.g. 299.5 - 273.15 = 26.35)
//  AMBIENT_DEFAULT = Voltage measurement at default temperature.
//  OFFSET_DEFAULT  = Default ADC offset at 1v.
//
// ****************************************************************************
#define AM_HAL_ADC_CALIB_TEMP_DEFAULT       (299.5F)
#define AM_HAL_ADC_CALIB_AMBIENT_DEFAULT    (1.02809F)
#define AM_HAL_ADC_CALIB_ADC_OFFSET_DEFAULT (-0.004281F)

//*****************************************************************************
//
// Forward declarations.
//
//*****************************************************************************
void set_Adv_Name(void); //BLE_exaample_funcs.cpp fn to parse s_AdvName string to BLE
void exactle_stack_init(void);
void scheduler_timer_init(void);
void update_scheduler_timers(void);
void set_next_wakeup(void);
void UserRequestReceived(uint8_t * buf, uint16_t len);
void enable_print_interface(void);
extern void timer_isr(void);

#ifdef __cplusplus
extern "C" {
#endif

bool AmdtpSendData(uint8_t *buf, uint16_t len);

#ifdef __cplusplus
};
#endif


#endif // _BLE_EXAMPLE_H_
