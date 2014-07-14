// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// User specific config file.  Any items listed in config.h can be overridden here.

// If you used to define your CONFIG_APM_HARDWARE setting here, it is no longer
// valid! You should switch to using a HAL_BOARD flag in your local config.mk.

//Custome 

// HARDWARE CONFIGURATION AND CONNECTIONS
# define CONGFIG_HAL_BOARD = HAL_BOARD_PX4 

// sensor types
# define SERIAL0_BAUD HAL_SERIAL0_BAUD_DEFAULT
# define HIL_MODE HIL_MODE_DISABLED
# define MAGNETOMETER ENABLED
# define CONFIG_SONAR_SOURCE SONAR_SOURCE_ANALOG_PIN

// FRAME_CONFIG
# define FRAME_CONFIG OCTA_QUAD_FRAME 

// ADC Disable
# define CONFIG_ADC DISABLED

// PWM control
# define RC_FAST_SPEED 490

// Barometer
# define CONFIG_BARO AP_BARO_BMP085

// Sonar
# define CONFIG_SONAR_SOURCE_ANALOG_PIN 13
# define CONFIG_SONAR ENABLED

# define SONAR_ALT_HEALTH_MAX 3
# define SONAR_RELIABLE_DISTANCE_PCT 0.60f
# define SONAR_GAIN_DEFAULT 0.8
# define THR_SURFACE_TRACKING_VELZ_MAX 150

// Channel 7 and 8 default options
# define CH7_OPTION AUX_SWITCH_DO_NOTHING
# define CH8_OPTION AUX_SWITCH_DO_NOTHING

// HIL_MODE
#define HIL_MODE HIL_MODE_DISABLED

// Serial port speeds
# define SERIAL0_BAUD 115200
# define SERIAL1_BAUD 57600
# define SERIAL2_BAUD 57600

// Battery monitoring
# define FS_BATT_VOLTAGE_DEFAULT 10.5f
# define FS_BATT_MAH_DEFAULT 0
# define BOARD_VOLTAGE_MIN 4.3f
# define FAILSAFE_GPS_TIMEOUT_MS 5000
# define GPS_HDOP_GOOD_DEFAULT 200
# define FS_GCS DISABLED
# define FS_GCS_TIMEOUT_MS 5000
#define FS_GCS_DISABLED 0
#define FS_GCS_ENABLED_ALWAYS_RTL 1
#define FS_GCS_ENABLED_CONTINUE_MISSION 2

// pre-arm check max velocity
# define PREARM_MAX_VELOCITY_CMS 50.0f

// MAGNETOMETER
# define MAGNETOMETER ENABLED

// expected magnetic field strength. pre-arm checks will fail if 50% higher or lower than this value
#define COMPASS_MAGFIELD_EXPECTED 530

// max compass offset length
#define COMPASS_OFFSETS_MAX 600

// OPTICAL_FLOW
# define OPTFLOW DISABLED

	#define OPTFLOW_ROLL_P 2.5f
	#define OPTFLOW_ROLL_I 0.5f
	#define OPTFLOW_ROLL_D 0.12f
	#define OPTFLOW_PITCH_P 2.5f
	#define OPTFLOW_PITCH_I 0.5f
	#define OPTFLOW_PITCH_D 0.12f
	#define OPTFLOW_IMAX 100

// Auto Tuning
# define AUTOTUNE_ENABLED ENABLED
	
// Crop Sprayer
# define EPM_ENABLED DISABLED

// Parachute release
# define PARACHUTE ENABLED

// Nav-Guided - allows external nav computer to control vehicle
# define NAV_GUIDED DISABLED


//////////////////////////////////////////////////////////////////////////////
// RADIO CONFIGURATION
//////////////////////////////////////////////////////////////////////////////

// FLIGHT_MODE
# define FLIGHT_MODE_1 STABILIZE
# define FLIGHT_MODE_2 ALT_HOLD
# define FLIGHT_MODE_3 ALT_HOLD
# define FLIGHT_MODE_4 ALT_HOLD
# define FLIGHT_MODE_5 ALT_HOLD
# define FLIGHT_MODE_6 ALT_HOLD

// Throttle Failsafe
# define FS_THR_VALUE_DEFAULT 975
# define LAND_SPEED 50 									// the descent speed for the final stage of landing in cm/s
# define LAND_START_ALT 1000 							// altitude in cm where land controller switches to slow rate of descent
# define LAND_DETECTOR_TRIGGER 50 						// number of 50hz iterations with near zero climb rate and low throttle that triggers landing complete.
# define LAND_REQUIRE_MIN_THROTTLE_TO_DISARM ENABLED
# define LAND_REPOSITION_DEFAULT 0 						// by default the pilot cannot override roll/pitch during landing
# define LAND_WITH_DELAY_MS 4000 						// default delay (in milliseconds) when a land-with-delay is triggered during a failsafe event

// CAMERA TRIGGER AND CONTROL
# define CAMERA ENABLED

// MOUNT (ANTENNA OR CAMERA)
# define MOUNT ENABLED
# define MOUNT2 DISABLED


//////////////////////////////////////////////////////////////////////////////
// Flight mode definitions
//////////////////////////////////////////////////////////////////////////////

// Acro Mode
# define ACRO_RP_P 4.5f
# define ACRO_YAW_P 4.5f
# define ACRO_LEVEL_MAX_ANGLE 3000
#define ACRO_BALANCE_ROLL 1.0f
#define ACRO_BALANCE_PITCH 1.0f

// Stabilize (angle controller) gains
# define STABILIZE_ROLL_P 4.5f
# define STABILIZE_PITCH_P 4.5f
# define STABILIZE_YAW_P 4.5f

// RTL Mode
# define RTL_ALT_FINAL 0 // the altitude the vehicle will move to as the final stage of Returning to Launch. Set to zero to land.
# define RTL_ALT 1500 // default alt to return to home in cm, 0 = Maintain current altitude
# define RTL_LOITER_TIME 5000 // Time (in milliseconds) to loiter above home before begining final descent

// AUTO Mode
# define WP_YAW_BEHAVIOR_DEFAULT WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL
# define AUTO_YAW_SLEW_RATE 60 // degrees/sec
# define YAW_LOOK_AHEAD_MIN_SPEED 100 // minimum ground speed in cm/s required before copter is aimed at ground course

// Super Simple mode
# define SUPER_SIMPLE_RADIUS 1000

// Stabilize Rate Control
# define ROLL_PITCH_INPUT_MAX 4500 // roll, pitch input range
# define DEFAULT_ANGLE_MAX 4500 // ANGLE_MAX parameters default value
# define ANGLE_RATE_MAX 18000 // default maximum rotation rate in roll/pitch axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes

// Rate controller gains
# define RATE_ROLL_P 0.150f
# define RATE_ROLL_I 0.100f
# define RATE_ROLL_D 0.004f
# define RATE_ROLL_IMAX 500
# define RATE_PITCH_P 0.150f
# define RATE_PITCH_I 0.100f
# define RATE_PITCH_D 0.004f
# define RATE_PITCH_IMAX 500
# define RATE_YAW_P 0.200f
# define RATE_YAW_I 0.020f
# define RATE_YAW_D 0.000f
# define RATE_YAW_IMAX 800

// Loiter position control gains
# define LOITER_POS_P 1.0f
# define LOITER_RATE_I 0.5f
# define LOITER_RATE_D 0.0f
# define LOITER_RATE_IMAX 1000 // maximum acceleration from I term build-up in cm/s/s

// PosHold parameter defaults
# define POSHOLD_ENABLED ENABLED // PosHold flight mode enabled by default
# define POSHOLD_BRAKE_RATE_DEFAULT 8 // default POSHOLD_BRAKE_RATE param value. Rotation rate during braking in deg/sec
# define POSHOLD_BRAKE_ANGLE_DEFAULT 3000 // default POSHOLD_BRAKE_ANGLE param value. Max lean angle during braking in centi-degrees

// Throttle control gains
# define THROTTLE_CRUISE 450 // default estimate of throttle required for vehicle to maintain a hover
# define THR_MID_DEFAULT 500 // Throttle output (0 ~ 1000) when throttle stick is in mid position
# define THR_MIN_DEFAULT 130 // minimum throttle sent to the motors when armed and pilot throttle above zero
# define THROTTLE_IN_DEADBAND 100 // the throttle input channel's deadband in PWM
# define ALT_HOLD_P 1.0f

// RATE control
# define THROTTLE_RATE_P 6.0f

// default maximum vertical velocity and acceleration the pilot may request
# define PILOT_VELZ_MAX 250 // maximum vertical velocity in cm/s
# define PILOT_ACCEL_Z_DEFAULT 250 // vertical acceleration in cm/s/s while altitude is under pilot control

// max distance in cm above or below current location that will be used for the alt target when transitioning to alt-hold mode
# define ALT_HOLD_INIT_MAX_OVERSHOOT 200

// the acceleration used to define the distance-velocity curve
# define ALT_HOLD_ACCEL_MAX 250 // if you change this you must also update the duplicate declaration in AC_WPNav.h

// Throttle Accel control
# define THROTTLE_ACCEL_P 0.75f
# define THROTTLE_ACCEL_I 1.50f
# define THROTTLE_ACCEL_D 0.0f
# define THROTTLE_ACCEL_IMAX 500

// Dataflash logging control
# define LOGGING_ENABLED ENABLED
# define DEFAULT_LOG_BITMASK \

// AP_Limits Defaults
#define AC_FENCE ENABLED
#define AC_RALLY ENABLED

// Developer Items
# define CLI_ENABLED ENABLED
#define FIRMWARE_STRING THISFIRMWARE
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"


// User Hooks : For User Developed code that you wish to run
// Put your variable definitions into the UserVariables.h file (or another file name and then change the #define below).
//#define USERHOOK_VARIABLES "UserVariables.h"
// Put your custom code into the UserCode.pde with function names matching those listed below and ensure the appropriate #define below is uncommented below
//#define USERHOOK_INIT userhook_init();                      // for code to be run once at startup
//#define USERHOOK_FASTLOOP userhook_FastLoop();            // for code to be run at 100hz
//#define USERHOOK_50HZLOOP userhook_50Hz();                  // for code to be run at 50hz
//#define USERHOOK_MEDIUMLOOP userhook_MediumLoop();        // for code to be run at 10hz
//#define USERHOOK_SLOWLOOP userhook_SlowLoop();            // for code to be run at 3.3hz
//#define USERHOOK_SUPERSLOWLOOP userhook_SuperSlowLoop();  // for code to be run at 1hz
