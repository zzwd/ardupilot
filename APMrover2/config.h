//
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
// WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
//
//  DO NOT EDIT this file to adjust your configuration.  Create your own
//  APM_Config.h and use APM_Config.h.example as a reference.
//
// WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
///
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
// Default and automatic configuration details.
//
// Notes for maintainers:
//
// - Try to keep this file organised in the same order as APM_Config.h.example
//
#pragma once

#include "defines.h"

///
/// DO NOT EDIT THIS INCLUDE - if you want to make a local change, make that
/// change in your local copy of APM_Config.h.
///
#include "APM_Config.h"  // <== THIS INCLUDE, DO NOT EDIT IT. EVER.
///
/// DO NOT EDIT THIS INCLUDE - if you want to make a local change, make that
/// change in your local copy of APM_Config.h.
///

//////////////////////////////////////////////////////////////////////////////
// sensor types

//////////////////////////////////////////////////////////////////////////////
// HIL_MODE                                 OPTIONAL

#ifndef HIL_MODE
  #define HIL_MODE HIL_MODE_DISABLED
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
  #define BATTERY_PIN_1     1
  #define CURRENT_PIN_1     2
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4
  #define BATTERY_PIN_1    -1
  #define CURRENT_PIN_1    -1
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
  #define BATTERY_PIN_1    -1
  #define CURRENT_PIN_1    -1
#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
  #define BATTERY_PIN_1    -1
  #define CURRENT_PIN_1    -1
#endif

#ifndef MAV_SYSTEM_ID
  #define MAV_SYSTEM_ID    1
#endif

#ifndef ARM_DELAY_MS
  #define ARM_DELAY_MS  2000
#endif

//////////////////////////////////////////////////////////////////////////////
// FrSky telemetry support
//

#ifndef FRSKY_TELEM_ENABLED
  #define FRSKY_TELEM_ENABLED ENABLED
#endif


#ifndef CH7_OPTION
  #define CH7_OPTION CH7_SAVE_WP
#endif

//////////////////////////////////////////////////////////////////////////////
//  MAGNETOMETER
#ifndef MAGNETOMETER
  #define MAGNETOMETER ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// MODE
// MODE_CHANNEL
//
#ifndef MODE_CHANNEL
  #define MODE_CHANNEL    8
#endif
#if (MODE_CHANNEL != 5) && (MODE_CHANNEL != 6) && (MODE_CHANNEL != 7) && (MODE_CHANNEL != 8)
  #error XXX
  #error XXX You must set MODE_CHANNEL to 5, 6, 7 or 8
  #error XXX
#endif


//////////////////////////////////////////////////////////////////////////////
//  VISUAL ODOMETRY
#ifndef VISUAL_ODOMETRY_ENABLED
# define VISUAL_ODOMETRY_ENABLED ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// STARTUP BEHAVIOUR
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// GROUND_START_DELAY
//
#ifndef GROUND_START_DELAY
  #define GROUND_START_DELAY    0
#endif

//////////////////////////////////////////////////////////////////////////////
// MOUNT (ANTENNA OR CAMERA)
//
#ifndef MOUNT
  #define MOUNT ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// CAMERA control
//
#ifndef CAMERA
  #define CAMERA ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// NAVL1
//
#ifndef NAVL1
  #define NAVL1_PERIOD    8
#endif

//////////////////////////////////////////////////////////////////////////////
// CRUISE_SPEED default
//
#ifndef CRUISE_SPEED
  #define CRUISE_SPEED    2  // in m/s
#endif

#ifndef TURN_GAIN
  #define TURN_GAIN       5
#endif

//////////////////////////////////////////////////////////////////////////////
// Dataflash logging control
//
#ifndef LOGGING_ENABLED
  #define LOGGING_ENABLED ENABLED
#endif

#define DEFAULT_LOG_BITMASK    0xffff


//////////////////////////////////////////////////////////////////////////////
// Developer Items
//

// if RESET_SWITCH_CH is not zero, then this is the PWM value on
// that channel where we reset the control mode to the current switch
// position (to for example return to switched mode after failsafe or
// fence breach)
#ifndef RESET_SWITCH_CHAN_PWM
  #define RESET_SWITCH_CHAN_PWM    1750
#endif

#ifndef ADVANCED_FAILSAFE
  #define ADVANCED_FAILSAFE DISABLED
#endif

#ifndef STATS_ENABLED
 # define STATS_ENABLED ENABLED
#endif

#ifndef DEVO_TELEM_ENABLED
#if HAL_MINIMIZE_FEATURES
 #define DEVO_TELEM_ENABLED DISABLED
#else
 #define DEVO_TELEM_ENABLED ENABLED
#endif
#endif
