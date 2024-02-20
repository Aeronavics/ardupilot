//
#pragma once

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
#include "defines.h"

///
/// DO NOT EDIT THIS INCLUDE - if you want to make a local change, make that
/// change in your local copy of APM_Config.h.
///
#include "APM_Config.h"


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// HARDWARE CONFIGURATION AND CONNECTIONS
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifndef CONFIG_HAL_BOARD
#error CONFIG_HAL_BOARD must be defined to build ArduCopter
#endif

#ifndef ARMING_DELAY_SEC
    # define ARMING_DELAY_SEC 2.0f
#endif

//////////////////////////////////////////////////////////////////////////////
// FRAME_CONFIG
//
#ifndef FRAME_CONFIG
 # define FRAME_CONFIG   MULTICOPTER_FRAME
#endif

/////////////////////////////////////////////////////////////////////////////////
// TradHeli defaults
#if FRAME_CONFIG == HELI_FRAME
  # define RC_FAST_SPEED                        125
  # define WP_YAW_BEHAVIOR_DEFAULT              WP_YAW_BEHAVIOR_LOOK_AHEAD
#endif

//////////////////////////////////////////////////////////////////////////////
// PWM control
// default RC speed in Hz
#ifndef RC_FAST_SPEED
   #   define RC_FAST_SPEED 490
#endif

//////////////////////////////////////////////////////////////////////////////
// Rangefinder
//

#ifndef RANGEFINDER_ENABLED
 # define RANGEFINDER_ENABLED ENABLED
#endif

#ifndef RANGEFINDER_HEALTH_MAX
 # define RANGEFINDER_HEALTH_MAX 3          // number of good reads that indicates a healthy rangefinder
#endif

#ifndef RANGEFINDER_TIMEOUT_MS
# define RANGEFINDER_TIMEOUT_MS 1000        // rangefinder filter reset if no updates from sensor in 1 second
#endif

#ifndef RANGEFINDER_FILT_DEFAULT
 # define RANGEFINDER_FILT_DEFAULT 0.5f     // filter for rangefinder distance
#endif

#ifndef SURFACE_TRACKING_TIMEOUT_MS
 # define SURFACE_TRACKING_TIMEOUT_MS  1000 // surface tracking target alt will reset to current rangefinder alt after this many milliseconds without a good rangefinder alt
#endif

#ifndef RANGEFINDER_TILT_CORRECTION         // by disable tilt correction for use of range finder data by EKF
 # define RANGEFINDER_TILT_CORRECTION ENABLED
#endif

#ifndef RANGEFINDER_GLITCH_ALT_CM
 # define RANGEFINDER_GLITCH_ALT_CM  200      // amount of rangefinder change to be considered a glitch
#endif

#ifndef RANGEFINDER_GLITCH_NUM_SAMPLES
 # define RANGEFINDER_GLITCH_NUM_SAMPLES  3   // number of rangefinder glitches in a row to take new reading
#endif

#ifndef MAV_SYSTEM_ID
 # define MAV_SYSTEM_ID          1
#endif

// prearm GPS hdop check
#ifndef GPS_HDOP_GOOD_DEFAULT
 # define GPS_HDOP_GOOD_DEFAULT         140     // minimum hdop that represents a good position.  used during pre-arm checks if fence is enabled
#endif

// missing terrain data failsafe
#ifndef FS_TERRAIN_TIMEOUT_MS
 #define FS_TERRAIN_TIMEOUT_MS          5000     // 5 seconds of missing terrain data will trigger failsafe (RTL)
#endif

// pre-arm baro vs inertial nav max alt disparity
#ifndef PREARM_MAX_ALT_DISPARITY_CM
 # define PREARM_MAX_ALT_DISPARITY_CM       100     // barometer and inertial nav altitude must be within this many centimeters
#endif

//////////////////////////////////////////////////////////////////////////////
//  EKF Failsafe
#ifndef FS_EKF_ACTION_DEFAULT
 # define FS_EKF_ACTION_DEFAULT         FS_EKF_ACTION_LAND  // EKF failsafe triggers land by default
#endif
#ifndef FS_EKF_THRESHOLD_DEFAULT
 # define FS_EKF_THRESHOLD_DEFAULT      0.8f    // EKF failsafe's default compass and velocity variance threshold above which the EKF failsafe will be triggered
#endif

#ifndef EKF_ORIGIN_MAX_ALT_KM
 # define EKF_ORIGIN_MAX_ALT_KM         50   // EKF origin and home must be within 50km vertically
#endif

//////////////////////////////////////////////////////////////////////////////
//  Auto Tuning
#ifndef AUTOTUNE_ENABLED
 # define AUTOTUNE_ENABLED  AC_AVOID_DISABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Precision Landing with companion computer or IRLock sensor
#ifndef PRECISION_LANDING
 # define PRECISION_LANDING ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Parachute release
#ifndef PARACHUTE
 # define PARACHUTE HAL_PARACHUTE_ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Nav-Guided - allows external nav computer to control vehicle
#ifndef NAV_GUIDED
 # define NAV_GUIDED    !HAL_MINIMIZE_FEATURES
#endif

//////////////////////////////////////////////////////////////////////////////
// Acro - fly vehicle in acrobatic mode
#ifndef MODE_ACRO_ENABLED
# define MODE_ACRO_ENABLED DISABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Auto mode - allows vehicle to trace waypoints and perform automated actions
#ifndef MODE_AUTO_ENABLED
# define MODE_AUTO_ENABLED ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Brake mode - bring vehicle to stop
#ifndef MODE_BRAKE_ENABLED
# define MODE_BRAKE_ENABLED DISABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Circle - fly vehicle around a central point
#ifndef MODE_CIRCLE_ENABLED
# define MODE_CIRCLE_ENABLED ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Drift - fly vehicle in altitude-held, coordinated-turn mode
#ifndef MODE_DRIFT_ENABLED
# define MODE_DRIFT_ENABLED DISABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// flip - fly vehicle in flip in pitch and roll direction mode
#ifndef MODE_FLIP_ENABLED
# define MODE_FLIP_ENABLED DISABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Follow - follow another vehicle or GCS
#ifndef MODE_FOLLOW_ENABLED
# define MODE_FOLLOW_ENABLED !HAL_MINIMIZE_FEATURES
#endif

//////////////////////////////////////////////////////////////////////////////
// Guided mode - control vehicle's position or angles from GCS
#ifndef MODE_GUIDED_ENABLED
# define MODE_GUIDED_ENABLED ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// GuidedNoGPS mode - control vehicle's angles from GCS
#ifndef MODE_GUIDED_NOGPS_ENABLED
# define MODE_GUIDED_NOGPS_ENABLED !HAL_MINIMIZE_FEATURES
#endif

//////////////////////////////////////////////////////////////////////////////
// Loiter mode - allows vehicle to hold global position
#ifndef MODE_LOITER_ENABLED
# define MODE_LOITER_ENABLED ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Position Hold - enable holding of global position
#ifndef MODE_POSHOLD_ENABLED
# define MODE_POSHOLD_ENABLED DISABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// RTL - Return To Launch
#ifndef MODE_RTL_ENABLED
# define MODE_RTL_ENABLED ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// SmartRTL - allows vehicle to retrace a (loop-eliminated) breadcrumb home
#ifndef MODE_SMARTRTL_ENABLED
# define MODE_SMARTRTL_ENABLED ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Sport - fly vehicle in rate-controlled (earth-frame) mode
#ifndef MODE_SPORT_ENABLED
# define MODE_SPORT_ENABLED ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// System ID - conduct system identification tests on vehicle
#ifndef MODE_SYSTEMID_ENABLED
# define MODE_SYSTEMID_ENABLED DISABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Throw - fly vehicle after throwing it in the air
#ifndef MODE_THROW_ENABLED
# define MODE_THROW_ENABLED DISABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// ZigZag - allow vehicle to fly in a zigzag manner with predefined point A B
#ifndef MODE_ZIGZAG_ENABLED
# define MODE_ZIGZAG_ENABLED DISABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Turtle - allow vehicle to be flipped over after a crash
#ifndef MODE_TURTLE_ENABLED
# define MODE_TURTLE_ENABLED DISABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Flowhold - use optical flow to hover in place
#ifndef MODE_FLOWHOLD_ENABLED
# define MODE_FLOWHOLD_ENABLED !HAL_MINIMIZE_FEATURES && AP_OPTICALFLOW_ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Weathervane - allow vehicle to yaw into wind
#ifndef WEATHERVANE_ENABLED
# define WEATHERVANE_ENABLED !HAL_MINIMIZE_FEATURES
#endif

//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Autorotate - autonomous auto-rotation - helicopters only
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    #if FRAME_CONFIG == HELI_FRAME
        #ifndef MODE_AUTOROTATE_ENABLED
        # define MODE_AUTOROTATE_ENABLED !HAL_MINIMIZE_FEATURES
        #endif
    #else
        # define MODE_AUTOROTATE_ENABLED DISABLED
    #endif
#else
    # define MODE_AUTOROTATE_ENABLED DISABLED
#endif

//////////////////////////////////////////////////////////////////////////////

// Beacon support - support for local positioning systems
#ifndef BEACON_ENABLED
# define BEACON_ENABLED !HAL_MINIMIZE_FEATURES
#endif

//////////////////////////////////////////////////////////////////////////////
// RADIO CONFIGURATION
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// FLIGHT_MODE
//

#ifndef FLIGHT_MODE_1
 # define FLIGHT_MODE_1                  Mode::Number::STABILIZE
#endif
#ifndef FLIGHT_MODE_2
 # define FLIGHT_MODE_2                  Mode::Number::STABILIZE
#endif
#ifndef FLIGHT_MODE_3
 # define FLIGHT_MODE_3                  Mode::Number::STABILIZE
#endif
#ifndef FLIGHT_MODE_4
 # define FLIGHT_MODE_4                  Mode::Number::STABILIZE
#endif
#ifndef FLIGHT_MODE_5
 # define FLIGHT_MODE_5                  Mode::Number::STABILIZE
#endif
#ifndef FLIGHT_MODE_6
 # define FLIGHT_MODE_6                  Mode::Number::STABILIZE
#endif


//////////////////////////////////////////////////////////////////////////////
// Throttle Failsafe
//
#ifndef FS_THR_VALUE_DEFAULT
 # define FS_THR_VALUE_DEFAULT             975
#endif

//////////////////////////////////////////////////////////////////////////////
// Takeoff
//
#ifndef PILOT_TKOFF_ALT_DEFAULT
 # define PILOT_TKOFF_ALT_DEFAULT           0     // default final alt above home for pilot initiated takeoff
#endif


//////////////////////////////////////////////////////////////////////////////
// Landing
//
#ifndef LAND_SPEED
 # define LAND_SPEED    50          // the descent speed for the final stage of landing in cm/s
#endif
#ifndef LAND_REPOSITION_DEFAULT
 # define LAND_REPOSITION_DEFAULT   1   // by default the pilot can override roll/pitch during landing
#endif
#ifndef LAND_WITH_DELAY_MS
 # define LAND_WITH_DELAY_MS        4000    // default delay (in milliseconds) when a land-with-delay is triggered during a failsafe event
#endif
#ifndef LAND_CANCEL_TRIGGER_THR
 # define LAND_CANCEL_TRIGGER_THR   700     // land is cancelled by input throttle above 700
#endif
#ifndef LAND_RANGEFINDER_MIN_ALT_CM
#define LAND_RANGEFINDER_MIN_ALT_CM 200
#endif

//////////////////////////////////////////////////////////////////////////////
// Landing Detector
//
#ifndef LAND_DETECTOR_TRIGGER_SEC
 # define LAND_DETECTOR_TRIGGER_SEC         1.0f    // number of seconds to detect a landing
#endif
#ifndef LAND_AIRMODE_DETECTOR_TRIGGER_SEC
 # define LAND_AIRMODE_DETECTOR_TRIGGER_SEC 3.0f    // number of seconds to detect a landing in air mode
#endif
#ifndef LAND_DETECTOR_MAYBE_TRIGGER_SEC
 # define LAND_DETECTOR_MAYBE_TRIGGER_SEC   0.2f    // number of seconds that means we might be landed (used to reset horizontal position targets to prevent tipping over)
#endif
#ifndef LAND_DETECTOR_ACCEL_LPF_CUTOFF
# define LAND_DETECTOR_ACCEL_LPF_CUTOFF     1.0f    // frequency cutoff of land detector accelerometer filter
#endif
#ifndef LAND_DETECTOR_ACCEL_MAX
# define LAND_DETECTOR_ACCEL_MAX            1.0f    // vehicle acceleration must be under 1m/s/s
#endif

//////////////////////////////////////////////////////////////////////////////
// Flight mode definitions
//

// Acro Mode
#ifndef ACRO_LEVEL_MAX_ANGLE
 # define ACRO_LEVEL_MAX_ANGLE      3000 // maximum lean angle in trainer mode measured in centidegrees
#endif

#ifndef ACRO_LEVEL_MAX_OVERSHOOT
 # define ACRO_LEVEL_MAX_OVERSHOOT  1000 // maximum overshoot angle in trainer mode when full roll or pitch stick is held in centidegrees
#endif

#ifndef ACRO_BALANCE_ROLL
 #define ACRO_BALANCE_ROLL          1.0f
#endif

#ifndef ACRO_BALANCE_PITCH
 #define ACRO_BALANCE_PITCH         1.0f
#endif

#ifndef ACRO_RP_EXPO_DEFAULT
 #define ACRO_RP_EXPO_DEFAULT       0.3f    // ACRO roll and pitch expo parameter default
#endif

#ifndef ACRO_Y_EXPO_DEFAULT
 #define ACRO_Y_EXPO_DEFAULT        0.0f    // ACRO yaw expo parameter default
#endif

#ifndef ACRO_THR_MID_DEFAULT
 #define ACRO_THR_MID_DEFAULT       0.0f
#endif

#ifndef ACRO_RP_RATE_DEFAULT
 #define ACRO_RP_RATE_DEFAULT      360      // ACRO roll and pitch rotation rate parameter default in deg/s
#endif

#ifndef ACRO_Y_RATE_DEFAULT
 #define ACRO_Y_RATE_DEFAULT       202.5    // ACRO yaw rotation rate parameter default in deg/s
#endif

// RTL Mode
#ifndef RTL_ALT_FINAL
 # define RTL_ALT_FINAL             0       // the altitude, in cm, the vehicle will move to as the final stage of Returning to Launch.  Set to zero to land.
#endif

#ifndef RTL_ALT
 # define RTL_ALT                   1500    // default alt to return to home in cm, 0 = Maintain current altitude
#endif

#ifndef RTL_ALT_MIN
 # define RTL_ALT_MIN               200     // min height above ground for RTL (i.e 2m)
#endif

#ifndef RTL_CLIMB_MIN_DEFAULT
 # define RTL_CLIMB_MIN_DEFAULT     0       // vehicle will always climb this many cm as first stage of RTL
#endif

#ifndef RTL_ABS_MIN_CLIMB
 # define RTL_ABS_MIN_CLIMB         250     // absolute minimum initial climb
#endif

#ifndef RTL_CONE_SLOPE_DEFAULT
 # define RTL_CONE_SLOPE_DEFAULT    3.0f    // slope of RTL cone (height / distance). 0 = No cone
#endif

#ifndef RTL_MIN_CONE_SLOPE
 # define RTL_MIN_CONE_SLOPE        0.5f    // minimum slope of cone
#endif

#ifndef RTL_LOITER_TIME
 # define RTL_LOITER_TIME           5000    // Time (in milliseconds) to loiter above home before beginning final descent
#endif

// AUTO Mode
#ifndef WP_YAW_BEHAVIOR_DEFAULT
 # define WP_YAW_BEHAVIOR_DEFAULT   WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL
#endif

#ifndef YAW_LOOK_AHEAD_MIN_SPEED
 # define YAW_LOOK_AHEAD_MIN_SPEED  100             // minimum ground speed in cm/s required before copter is aimed at ground course
#endif

// Super Simple mode
#ifndef SUPER_SIMPLE_RADIUS
 # define SUPER_SIMPLE_RADIUS       1000
#endif

//////////////////////////////////////////////////////////////////////////////
// Stabilize Rate Control
//
#ifndef ROLL_PITCH_YAW_INPUT_MAX
 # define ROLL_PITCH_YAW_INPUT_MAX      4500        // roll, pitch and yaw input range
#endif
#ifndef DEFAULT_ANGLE_MAX
 # define DEFAULT_ANGLE_MAX         3000            // ANGLE_MAX parameters default value
#endif

//////////////////////////////////////////////////////////////////////////////
// Stop mode defaults
//
#ifndef BRAKE_MODE_SPEED_Z
 # define BRAKE_MODE_SPEED_Z     250 // z-axis speed in cm/s in Brake Mode
#endif
#ifndef BRAKE_MODE_DECEL_RATE
 # define BRAKE_MODE_DECEL_RATE  750 // acceleration rate in cm/s/s in Brake Mode
#endif

//////////////////////////////////////////////////////////////////////////////
// PosHold parameter defaults
//
#ifndef POSHOLD_BRAKE_RATE_DEFAULT
 # define POSHOLD_BRAKE_RATE_DEFAULT    8       // default POSHOLD_BRAKE_RATE param value.  Rotation rate during braking in deg/sec
#endif
#ifndef POSHOLD_BRAKE_ANGLE_DEFAULT
 # define POSHOLD_BRAKE_ANGLE_DEFAULT   3000    // default POSHOLD_BRAKE_ANGLE param value.  Max lean angle during braking in centi-degrees
#endif

//////////////////////////////////////////////////////////////////////////////
// Pilot control defaults
//

#ifndef THR_DZ_DEFAULT
# define THR_DZ_DEFAULT         100             // the deadzone above and below mid throttle while in althold or loiter
#endif

// default maximum vertical velocity and acceleration the pilot may request
#ifndef PILOT_VELZ_MAX
 # define PILOT_VELZ_MAX    250     // maximum vertical velocity in cm/s
#endif
#ifndef PILOT_ACCEL_Z_DEFAULT
 # define PILOT_ACCEL_Z_DEFAULT 250 // vertical acceleration in cm/s/s while altitude is under pilot control
#endif

#ifndef PILOT_Y_RATE_DEFAULT
 # define PILOT_Y_RATE_DEFAULT  202.5   // yaw rotation rate parameter default in deg/s for all mode except ACRO
#endif
#ifndef PILOT_Y_EXPO_DEFAULT
 # define PILOT_Y_EXPO_DEFAULT  0.0     // yaw expo parameter default for all mode except ACRO
#endif

#ifndef AUTO_DISARMING_DELAY
# define AUTO_DISARMING_DELAY  10
#endif

//////////////////////////////////////////////////////////////////////////////
// Throw mode configuration
//
#ifndef THROW_HIGH_SPEED
# define THROW_HIGH_SPEED       500.0f  // vehicle much reach this total 3D speed in cm/s (or be free falling)
#endif
#ifndef THROW_VERTICAL_SPEED
# define THROW_VERTICAL_SPEED   50.0f   // motors start when vehicle reaches this total 3D speed in cm/s
#endif

//////////////////////////////////////////////////////////////////////////////
// Logging control
//
#ifndef LOGGING_ENABLED
 # define LOGGING_ENABLED                ENABLED
#endif

// Default logging bitmask
#ifndef DEFAULT_LOG_BITMASK
 # define DEFAULT_LOG_BITMASK \
    MASK_LOG_ATTITUDE_MED | \
    MASK_LOG_GPS | \
    MASK_LOG_PM | \
    MASK_LOG_CTUN | \
    MASK_LOG_NTUN | \
    MASK_LOG_RCIN | \
    MASK_LOG_IMU | \
    MASK_LOG_CMD | \
    MASK_LOG_CURRENT | \
    MASK_LOG_RCOUT | \
    MASK_LOG_OPTFLOW | \
    MASK_LOG_COMPASS | \
    MASK_LOG_CAMERA | \
    MASK_LOG_MOTBATT
#endif

//////////////////////////////////////////////////////////////////////////////
// Fence, Rally and Terrain and AC_Avoidance defaults
//

#ifndef AC_AVOID_ENABLED
 #define AC_AVOID_ENABLED   ENABLED
#endif

#ifndef AC_OAPATHPLANNER_ENABLED
 #define AC_OAPATHPLANNER_ENABLED   !HAL_MINIMIZE_FEATURES
#endif

#if MODE_FOLLOW_ENABLED && !AC_AVOID_ENABLED
  #error Follow Mode relies on AC_AVOID which is disabled
#endif

#if MODE_AUTO_ENABLED && !MODE_GUIDED_ENABLED
  #error ModeAuto requires ModeGuided which is disabled
#endif

#if MODE_AUTO_ENABLED && !MODE_CIRCLE_ENABLED
  #error ModeAuto requires ModeCircle which is disabled
#endif

#if MODE_AUTO_ENABLED && !MODE_RTL_ENABLED
  #error ModeAuto requires ModeRTL which is disabled
#endif

#if FRAME_CONFIG == HELI_FRAME && !MODE_ACRO_ENABLED
  #error Helicopter frame requires acro mode support which is disabled
#endif

#if MODE_SMARTRTL_ENABLED && !MODE_RTL_ENABLED
  #error SmartRTL requires ModeRTL which is disabled
#endif

#if HAL_ADSB_ENABLED && !MODE_GUIDED_ENABLED
  #error ADSB requires ModeGuided which is disabled
#endif

#if MODE_FOLLOW_ENABLED && !MODE_GUIDED_ENABLED
  #error Follow requires ModeGuided which is disabled
#endif

#if MODE_GUIDED_NOGPS_ENABLED && !MODE_GUIDED_ENABLED
  #error ModeGuided-NoGPS requires ModeGuided which is disabled
#endif

//////////////////////////////////////////////////////////////////////////////
// Developer Items
//

#ifndef ADVANCED_FAILSAFE
# define ADVANCED_FAILSAFE DISABLED
#endif

#ifndef CH_MODE_DEFAULT
 # define CH_MODE_DEFAULT   5
#endif

#ifndef TOY_MODE_ENABLED
#define TOY_MODE_ENABLED DISABLED
#endif

#if TOY_MODE_ENABLED && FRAME_CONFIG == HELI_FRAME
  #error Toy mode is not available on Helicopters
#endif

#ifndef STATS_ENABLED
 # define STATS_ENABLED ENABLED
#endif

#ifndef OSD_ENABLED
 #define OSD_ENABLED DISABLED
#endif

#ifndef HAL_FRAME_TYPE_DEFAULT
#define HAL_FRAME_TYPE_DEFAULT AP_Motors::MOTOR_FRAME_TYPE_X
#endif

#ifndef AC_CUSTOMCONTROL_MULTI_ENABLED
#define AC_CUSTOMCONTROL_MULTI_ENABLED FRAME_CONFIG == MULTICOPTER_FRAME && AP_CUSTOMCONTROL_ENABLED
#endif
