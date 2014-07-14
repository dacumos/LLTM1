// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// User specific config file.  Any items listed in config.h can be overridden here.

// If you used to define your CONFIG_APM_HARDWARE setting here, it is no longer
// valid! You should switch to using a HAL_BOARD flag in your local config.mk.

// HARDWARE CONFIGURATION AND CONNECTIONS
# define CONFIG_HAL_BOARD HAL_BOARD_PX4

// high power CPUs (Flymaple, PX4, Pixhawk, VRBrain)
# define MAIN_LOOP_RATE    400
# define MAIN_LOOP_SECONDS 0.0025
# define MAIN_LOOP_MICROS  2500

//FRAME_CONFIG
# define FRAME_CONFIG OCTA_QUAD_FRAME

// IMU Selection
# define CONFIG_IMU_TYPE   CONFIG_IMU_PX4

// ADC
# define CONFIG_ADC DISABLED

// PWM control
#   define RC_FAST_SPEED 490

// Barometer
# define CONFIG_BARO       AP_BARO_PX4

// Sonar
# define CONFIG_SONAR_SOURCE SONAR_SOURCE_ANALOG_PIN
# define CONFIG_SONAR_SOURCE_ANALOG_PIN 13
# define CONFIG_SONAR ENABLED

# define SONAR_ALT_HEALTH_MAX 3           						// number of good reads that indicates a healthy sonar				 
# define SONAR_RELIABLE_DISTANCE_PCT 0.60f 						// we trust the sonar out to 60% of it's maximum range
# define SONAR_GAIN_DEFAULT 0.8            						// gain for controlling how quickly sonar range adjusts target altitude (lower means slower reaction)
# define THR_SURFACE_TRACKING_VELZ_MAX 150 						// max vertical speed change while surface tracking with sonar	

// Channel 7 and 8 default options
# define CH7_OPTION            AUX_SWITCH_DO_NOTHING 
# define CH8_OPTION            AUX_SWITCH_DO_NOTHING

// HIL_MODE                                 
# define HIL_MODE        HIL_MODE_DISABLED

// Serial port speeds.
# define SERIAL0_BAUD                   115200
# define SERIAL1_BAUD                    57600
# define SERIAL2_BAUD                    57600

// Battery monitoring
# define FS_BATT_VOLTAGE_DEFAULT       10.50f 					// default battery voltage below which failsafe will be triggered
# define FS_BATT_MAH_DEFAULT             0 						// default battery capacity (in mah) below which failsafe will be triggered
# define BOARD_VOLTAGE_MIN             4.3f 					// min board voltage in volts for pre-arm checks
# define BOARD_VOLTAGE_MAX             5.8f						// max board voltage in volts for pre-arm checks

//GPS failsafe 
# define FS_GCS_TIMEOUT_MS             5000 					// gps failsafe triggers after 5 seconds with no GPS
# define GPS_HDOP_GOOD_DEFAULT         200     					// minimum hdop that represents a good position.  used during pre-arm checks if fence is enabled

// GPS failsafe
# define FS_GCS                        DISABLED

# define FS_GCS_TIMEOUT_MS             5000
# define FS_GCS_DISABLED                     0
# define FS_GCS_ENABLED_ALWAYS_RTL           1
# define FS_GCS_ENABLED_CONTINUE_MISSION     2

// pre-arm check max velocity
# define PREARM_MAX_VELOCITY_CMS           50.0f 				// vehicle must be travelling under 50cm/s before arming

//  MAGNETOMETER
# define MAGNETOMETER                   ENABLED

// expected magnetic field strength.
# define COMPASS_MAGFIELD_EXPECTED      530 					// pre arm will fail if mag field > 874 or < 185

// max compass offset length
# define COMPASS_OFFSETS_MAX          600 						// PX4 onboard compass has high offsets

//  OPTICAL_FLOW
# define OPTFLOW                       DISABLED

# define OPTFLOW_ROLL_P 2.5f
# define OPTFLOW_ROLL_I 0.5f
# define OPTFLOW_ROLL_D 0.12f
# define OPTFLOW_PITCH_P 2.5f
# define OPTFLOW_PITCH_I 0.5f
# define OPTFLOW_PITCH_D 0.12f
# define OPTFLOW_IMAX 100

//  Auto Tuning
# define AUTOTUNE_ENABLED  ENABLED

//  Crop Sprayer
# define SPRAYER  DISABLED

//	EPM cargo gripper
# define EPM_ENABLED DISABLED

// Parachute release
# define PARACHUTE DISABLED

// RADIO CONFIGURATION
	// FLIGHT_MODE
	# define FLIGHT_MODE_1                  STABILIZE
	# define FLIGHT_MODE_2                  ALT_HOLD
	# define FLIGHT_MODE_3                  ALT_HOLD
	# define FLIGHT_MODE_4                  ALT_HOLD
	# define FLIGHT_MODE_5                  ALT_HOLD
	# define FLIGHT_MODE_6                  ALT_HOLD

// Throttle Failsafe
# define FS_THR_VALUE_DEFAULT             975
# define LAND_SPEED    50  										// the descent speed for the final stage of landing in cm/s
# define LAND_START_ALT 1000  									// altitude in cm where land controller switches to slow rate of descent
# define LAND_DETECTOR_TRIGGER 50  								// number of 50hz iterations with near zero climb rate and low throttle that triggers landing complete.
# define LAND_REQUIRE_MIN_THROTTLE_TO_DISARM ENABLED

// CAMERA TRIGGER AND CONTROL
# define CAMERA        ENABLED

// MOUNT (ANTENNA OR CAMERA)
# define MOUNT         ENABLED
# define MOUNT2         DISABLED

// Flight mode definitions
	// Acro Mode
	# define ACRO_RP_P                 4.5f
	# define ACRO_YAW_P                4.5f
	# define ACRO_LEVEL_MAX_ANGLE      3000
	# define ACRO_BALANCE_ROLL          1.0f
	# define ACRO_BALANCE_PITCH         1.0f

	// Stabilize (angle controller) gains
	# define STABILIZE_ROLL_P          4.5f
	# define STABILIZE_PITCH_P         4.5f
	# define STABILIZE_YAW_P           4.5f

	// RTL Mode
	# define RTL_ALT_FINAL             0 						// the altitude the vehicle will move to as the final stage of Returning to Launch.  Set to zero to land.
	# define RTL_ALT 				    1500 					// default alt to return to home in cm, 0 = Maintain current altitude
	# define RTL_LOITER_TIME           5000 					// Time (in milliseconds) to loiter above home before begining final descent

	// AUTO Mode
	# define WP_YAW_BEHAVIOR_DEFAULT   WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL
	# define AUTO_YAW_SLEW_RATE    60    						// degrees/sec
	# define YAW_LOOK_AHEAD_MIN_SPEED  100  					// minimum ground speed in cm/s required before copter is aimed at ground course

	// Super Simple mode
	# define SUPER_SIMPLE_RADIUS       1000

// Stabilize Rate Control
# define ROLL_PITCH_INPUT_MAX      4500 						// roll, pitch input range
# define DEFAULT_ANGLE_MAX         4500   						// ANGLE_MAX parameters default value
# define ANGLE_RATE_MAX            18000 						// default maximum rotation rate in roll/pitch axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes

// Rate controller gains
# define RATE_ROLL_P        		0.150f 						
# define RATE_ROLL_I        		0.100f
# define RATE_ROLL_D        		0.004f
# define RATE_ROLL_IMAX         	500
# define RATE_PITCH_P       		0.150f
# define RATE_PITCH_I       		0.100f
# define RATE_PITCH_D       		0.004f
# define RATE_PITCH_IMAX        	500
# define RATE_YAW_P              	0.200f
# define RATE_YAW_I              	0.020f
# define RATE_YAW_D              	0.000f
# define RATE_YAW_IMAX            	800

// Loiter position control gains
# define LOITER_POS_P             	1.0f

// Loiter rate control gains
# define LOITER_RATE_P          	1.0f
# define LOITER_RATE_I          	0.5f
# define LOITER_RATE_D          	0.0f
# define LOITER_RATE_IMAX          1000 					// maximum acceleration from I term build-up in cm/s/s

// Hybrid parameter defaults
# define HYBRID_ENABLED                ENABLED              // hybrid flight mode enabled by default
# define HYBRID_BRAKE_RATE_DEFAULT     8                    // default HYBRID_BRAKE_RATE param value.  Rotation rate during braking in deg/sec
# define HYBRID_BRAKE_ANGLE_DEFAULT    3000 				// default HYBRID_BRAKE_ANGLE param value.  Max lean angle during braking in centi-degrees

// Throttle control gains
# define THROTTLE_CRUISE       450 					        // default estimate of throttle required for vehicle to maintain a hover		
# define THR_MID_DEFAULT       500 							// Throttle output (0 ~ 1000) when throttle stick is in mid position
# define THR_MIN_DEFAULT       130 							// minimum throttle sent to the motors when armed and pilot throttle above zero
# define THR_MAX_DEFAULT       1000 						// maximum throttle sent to the motors
# define THROTTLE_IN_DEADBAND    100 						// the throttle input channel's deadband in PWM
# define ALT_HOLD_P            1.0f

// RATE control
# define THROTTLE_RATE_P       6.0f

// default maximum vertical velocity and acceleration the pilot may request
# define PILOT_VELZ_MAX    250 								// maximum vertical velocity in cm/s
# define PILOT_ACCEL_Z_DEFAULT 250 							// vertical acceleration in cm/s/s while altitude is under pilot control

// max distance in cm above or below current location that will be used for the alt target when transitioning to alt-hold mode
# define ALT_HOLD_INIT_MAX_OVERSHOOT 200 					

// the acceleration used to define the distance-velocity curve
# define ALT_HOLD_ACCEL_MAX 250 							// if you change this you must also update the duplicate declaration in AC_WPNav.h

// Throttle Accel control
# define THROTTLE_ACCEL_P  0.75f 
# define THROTTLE_ACCEL_I  1.50f
# define THROTTLE_ACCEL_D 0.0f
# define THROTTLE_ACCEL_IMAX 500

// Dataflash logging control
# define LOGGING_ENABLED                ENABLED
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
    MASK_LOG_COMPASS | \
    MASK_LOG_CAMERA

// AP_Limits Defaults
// Enable/disable AP_Limits
# define AC_FENCE ENABLED
# define AC_RALLY   ENABLED

// Developer Items
// use this to completely disable the CLI
# define CLI_ENABLED           ENABLED

/*
  build a firmware version string.
  GIT_VERSION comes from Makefile builds
*/
# define FIRMWARE_STRING THISFIRMWARE
# define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"

							//when we add out own code this will be useful

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