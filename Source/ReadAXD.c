#include "ZComDef.h"
#include "OSAL.h"
//#include "sapi.h"
#include "hal_key.h"
#include "hal_led.h"
#include "hal_lcd.h"
#include "hal_adc.h"
#include "hal_mcu.h"
//#include "SimpleApp.h"

#include "readaxd.h"
#include "adxl345.h"
#include "AXD.h"

/*********************************************************************
 * CONSTANTS
 */

// Application States
#define APP_INIT                           0    // Initial state
#define APP_START                          1    // Sensor has joined network
#define APP_BOUND                          2    // Sensor is bound to collector

// Application osal event identifiers
// Bit mask of events ( from 0x0000 to 0x00FF )
#define MY_START_EVT                0x0001
#define MY_REPORT_TEMP_EVT          0x0002
#define MY_REPORT_BATT_EVT          0x0004
#define MY_FIND_COLLECTOR_EVT       0x0008
void zb_HandleOsalEvent( uint16 event )
{
  
}