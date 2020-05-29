#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "boards.h"
#include "macros.h"

#if ENABLED(USE_AUTOMATIC_VERSIONING)
  #include "_Version.h"
#else
  #include "Default_Version.h"
#endif

// User-specified version info of this build to display in [Pronterface, etc] terminal window during
// startup. Implementation of an idea by Prof Braino to inform user that any changes made to this
// build by the user have been successfully uploaded into firmware.
#define STRING_CONFIG_H_AUTHOR "(mirage335, TazStiff Defaults)" // Who made the changes.
//#define SHOW_BOOTSCREEN
#define STRING_SPLASH_LINE1 SHORT_BUILD_VERSION // will be shown during bootup in line 1
#define STRING_SPLASH_LINE2 STRING_DISTRIBUTION_DATE // will be shown during bootup in line 2

#define SERIAL_PORT 0
//#define BAUDRATE 57600
#define BAUDRATE 250000

#ifndef MOTHERBOARD
  #define MOTHERBOARD BOARD_RAMBO
#endif

#define CUSTOM_MACHINE_NAME "TazStiff"

#define POWER_SUPPLY 1

#define EXTRUDERS 2

// 1 is 100k thermistor - best choice for EPCOS 100k (4.7k pullup)l, recommended at http://www.reprapdiscount.com/hotends/67-hexagon-hotend-set.html
// 7 is 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup), recommended at https://www.lulzbot.com/store/parts/100k-honeywell-axial-thermistor, https://www.lulzbot.com/store/parts/24v-silicone-heater
#define TEMP_SENSOR_0 1
#define TEMP_SENSOR_1 1
#define TEMP_SENSOR_BED 7

// Actual temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 4  // (seconds)
#define TEMP_HYSTERESIS 6       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     4       // (degC) Window around target to start the residency timer x degC early.

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
// Set to -273.15 to isable.

#define HEATER_0_MINTEMP 0
#define HEATER_1_MINTEMP 0
#define BED_MINTEMP 0

#define HEATER_0_MAXTEMP 275
#define HEATER_1_MAXTEMP 275
#define BED_MAXTEMP 150

//Extruder PID settings. "In the case of multiple extruders (E0, E1, E2) these PID values are shared between the extruders..."
#define PIDTEMP
#define BANG_MAX 65 // limits current to nozzle while in bang-bang mode; 255=full current
#define PID_MAX 65 // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current

#if ENABLED(PIDTEMP)
	//#define PID_DEBUG // Sends debug data to the serial port.
	//#define PID_OPENLOOP 1 // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
	//#define SLOW_PWM_HEATERS // PWM with very low frequency (roughly 0.125Hz=8s) and minimum state time of approximately 1s useful for heaters driven by a relay
	//#define PID_PARAMS_PER_EXTRUDER // Uses separate PID parameters for each extruder (useful for mismatched extruders)
					// Set/get with gcode: M301 E[extruder number, 0-2]
	#define PID_FUNCTIONAL_RANGE 16 // If the temperature difference between the target temperature and the actual temperature
					// is more then PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.
	#define PID_INTEGRAL_DRIVE_MAX PID_MAX  //limit for the integral term
	#define K1 0.95 //smoothing factor within the PID
	#define PID_dT ((OVERSAMPLENR * 10.0)/(F_CPU / 64.0 / 256.0)) //sampling period of the temperature routine

	// Buda 2.0 on 24V
	//#define  DEFAULT_Kp 12
	//#define  DEFAULT_Ki .3
	//#define  DEFAULT_Kd 125
	
	//Hexagon on 24V, TazMega Custom PID
	//#define  DEFAULT_Kp 35
	//#define  DEFAULT_Ki .25
	//#define  DEFAULT_Kd 100

 //Hexagon on 24V, 5.5ohm Heater, Derived from TazMega Custom PID
  #define  DEFAULT_Kp 10
  #define  DEFAULT_Ki .25
  #define  DEFAULT_Kd 100
	
#endif

#define PREVENT_DANGEROUS_EXTRUDE
#define PREVENT_LENGTHY_EXTRUDE
#define EXTRUDE_MINTEMP 120
#define EXTRUDE_MAXLENGTH (X_MAX_LENGTH+Y_MAX_LENGTH)


//Bed PID settings.
#define PIDTEMPBED
//#define MAX_BED_POWER 250 // limits duty cycle to bed; 255=full current
//#define MAX_BED_POWER 96  //Low power setting,  90W, 90V emulated. Intended for use while sharing a UPS.
#define MAX_BED_POWER 184  //More aggressive UPS compatible option, ~170W, ~17V emulated.

#if ENABLED(PIDTEMPBED)
	#define PID_BED_INTEGRAL_DRIVE_MAX MAX_BED_POWER //limit for the integral term
	
	//24V 360W silicone heater from NPH on 3mm borosilicate (TAZ 2.2+) *WITH* 3/16" Aluminum Plate Carrier (Thermal Mass)
	#define  DEFAULT_bedKp 332.99
	#define  DEFAULT_bedKi 48.39
	#define  DEFAULT_bedKd 572.87
#endif


//Thermal Runaway protection.
// Extruders
#define THERMAL_RUNAWAY_PROTECTION_PERIOD 20 //in seconds
#define THERMAL_RUNAWAY_PROTECTION_HYSTERESIS 4 // in degree Celsius

// Parameters for the bed heater
#define THERMAL_RUNAWAY_PROTECTION_BED_PERIOD 20 //in seconds
#define THERMAL_RUNAWAY_PROTECTION_BED_HYSTERESIS 2 // in degree Celsius


//===========================================================================
//============================= Mechanical Settings =========================
//===========================================================================
// The pullups are needed if you directly connect a mechanical endstop between the signal and ground pins.
#define ENDSTOPPULLUP_XMAX
#define ENDSTOPPULLUP_YMAX
#define ENDSTOPPULLUP_ZMAX
#define ENDSTOPPULLUP_XMIN
#define ENDSTOPPULLUP_YMIN
#define ENDSTOPPULLUP_ZMIN

// Mechanical endstop with COM to ground and NC to Signal uses "false" here (most common setup).
const bool X_MIN_ENDSTOP_INVERTING = true;
const bool Y_MIN_ENDSTOP_INVERTING = true;
const bool Z_MIN_ENDSTOP_INVERTING = false;	//Probe configuration.
const bool X_MAX_ENDSTOP_INVERTING = true;
const bool Y_MAX_ENDSTOP_INVERTING = true;
const bool Z_MAX_ENDSTOP_INVERTING = true;

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders

// Disables axis when it's not being used.
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false // For all extruders
#define DISABLE_INACTIVE_EXTRUDER true //disable only inactive extruders and keep active extruder enabled

// If you motor turns to wrong direction, you can invert it here:
#define INVERT_X_DIR false
#define INVERT_Y_DIR true
#define INVERT_Z_DIR true
#define INVERT_E0_DIR true
#define INVERT_E1_DIR true

// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
//#define Z_HOME_DIR -1		//Home towards Zmin, required for Z_SAFE_HOMING.
#define Z_HOME_DIR 1

#define min_software_endstops true // If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops true  // If true, axis won't move to coordinates greater than the defined lengths below.

// Travel limits after homing (units are in mm)
#define X_MIN_POS 0
#define Y_MIN_POS 0
#define Z_MIN_POS 0
#define X_MAX_POS 379
#define Y_MAX_POS 340
#define Z_MAX_POS 190	//Roughly accurate with standard AlephObjects platform mounted.
//#define Z_MAX_POS 10	//Silly value for testing.
//#define Z_MAX_POS 180	//Safe value for probing.

#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)

#ifdef MANUAL_HOME_POSITIONS
	#define MANUAL_X_HOME_POS 0
	#define MANUAL_Y_HOME_POS 0
	#define MANUAL_Z_HOME_POS 0
	#define MANUAL_Z_HOME_POS 402 // Possibly distance between nozzle and print surface.
#endif


#define NUM_AXIS 4 // The axis order in all axis related arrays is X, Y, Z, E


#define HOMING_FEEDRATE {85*60, 85*60, 15*60, 0}  // set the homing speeds (mm/min)

//GT2 14 Tooth Pulley at 200step/rotation = 114.29
//GT2 16 Tooth Pulley at 200step/rotation = 100
//GT2 30 Tooth Pulley at 200step/rotation = 53.33
//AMCE 8mm Pitch Rod at 200step/rotation = 400
//TazMega extruders have been configured with 866 steps/mm.
#define DEFAULT_AXIS_STEPS_PER_UNIT   {114.29,114.29,400,750}

#define DEFAULT_MAX_FEEDRATE          {260, 260, 25, 20}
#define DEFAULT_MAX_ACCELERATION      {4000,4000,70,5000}	//XY tested at upwards of 12000mm/s^2 (~1.2G)

#define DEFAULT_ACCELERATION          3500
#define DEFAULT_RETRACT_ACCELERATION  3500
#define DEFAULT_TRAVEL_ACCELERATION   3500

// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).
// The offset has to be X=0, Y=0 for the extruder 0 hotend (default extruder).
// For the other hotends it is their distance from the extruder 0 hotend.
// #define EXTRUDER_OFFSET_X {0.0, 20.00} // (in mm) for each extruder, offset of the hotend on the X axis
// #define EXTRUDER_OFFSET_Y {0.0, 5.00}  // (in mm) for each extruder, offset of the hotend on the Y axis

// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
#define DEFAULT_XYJERK                15     // (mm/sec) reasonable upper value 35
#define DEFAULT_ZJERK                 0.5    // (mm/sec)
#define DEFAULT_EJERK                 2    // (mm/sec)







//===========================================================================
//=========================== Manual Bed Leveling ===========================
//===========================================================================

//#define MANUAL_BED_LEVELING  // Add display menu option for bed leveling.
//#define MESH_BED_LEVELING    // Enable mesh bed leveling.

#if ENABLED(MANUAL_BED_LEVELING)
  #define MBL_Z_STEP 0.025  // Step size while manually probing Z axis.
#endif  // MANUAL_BED_LEVELING

#if ENABLED(MESH_BED_LEVELING)
  #define MESH_MIN_X 90
  #define MESH_MAX_X (X_MAX_POS - MESH_MIN_X)
  #define MESH_MIN_Y 65
  #define MESH_MAX_Y (Y_MAX_POS - MESH_MIN_Y)
  #define MESH_NUM_X_POINTS 3  // Don't use more than 7 points per axis, implementation limited.
  #define MESH_NUM_Y_POINTS 3
  #define MESH_HOME_SEARCH_Z 4  // Z after Home, bed somewhere below but above 0.0.
#endif  // MESH_BED_LEVELING

//===========================================================================
//============================ Bed Auto Leveling ============================
//===========================================================================

// @section bedlevel

#define AUTO_BED_LEVELING_FEATURE // Delete the comment to enable (remove // at the start of the line)
//#define DEBUG_LEVELING_FEATURE
#define Z_MIN_PROBE_REPEATABILITY_TEST  // If not commented out, Z-Probe Repeatability test will be included if Auto Bed Leveling is Enabled.

#if ENABLED(AUTO_BED_LEVELING_FEATURE)

  // There are 2 different ways to specify probing locations:
  //
  // - "grid" mode
  //   Probe several points in a rectangular grid.
  //   You specify the rectangle and the density of sample points.
  //   This mode is preferred because there are more measurements.
  //
  // - "3-point" mode
  //   Probe 3 arbitrary points on the bed (that aren't colinear)
  //   You specify the XY coordinates of all 3 points.

  // Enable this to sample the bed in a grid (least squares solution).
  // Note: this feature generates 10KB extra code size.
  #define AUTO_BED_LEVELING_GRID

  #if ENABLED(AUTO_BED_LEVELING_GRID)

    #define LEFT_PROBE_BED_POSITION 70
    #define RIGHT_PROBE_BED_POSITION (X_MAX_POS - LEFT_PROBE_BED_POSITION)
    #define FRONT_PROBE_BED_POSITION (75 - 75)
    #define BACK_PROBE_BED_POSITION (Y_MAX_POS - FRONT_PROBE_BED_POSITION - 75 - 35)

    #define MIN_PROBE_EDGE 10 // The Z probe minimum square sides can be no smaller than this.

    // Set the number of grid points per dimension.
    // You probably don't need more than 3 (squared=9).
    #define AUTO_BED_LEVELING_GRID_POINTS 5

  #else  // !AUTO_BED_LEVELING_GRID

    // Arbitrary points to probe.
    // A simple cross-product is used to estimate the plane of the bed.
    #define ABL_PROBE_PT_1_X 15
    #define ABL_PROBE_PT_1_Y 180
    #define ABL_PROBE_PT_2_X 15
    #define ABL_PROBE_PT_2_Y 20
    #define ABL_PROBE_PT_3_X 170
    #define ABL_PROBE_PT_3_Y 20

  #endif // AUTO_BED_LEVELING_GRID

  // Offsets to the Z probe relative to the nozzle tip.
  // X and Y offsets must be integers.
  #define X_PROBE_OFFSET_FROM_EXTRUDER -70     // Z probe to nozzle X offset: -left  +right
  #define Y_PROBE_OFFSET_FROM_EXTRUDER -35     // Z probe to nozzle Y offset: -front +behind
  //#define Z_PROBE_OFFSET_FROM_EXTRUDER -1  // Z probe to nozzle Z offset: -below (always!)

  #define Z_PROBE_OFFSET_FROM_EXTRUDER -0.05   //@20C - Use slicer configuration to apply temperature dependent offsets.

  //#define Z_PROBE_OFFSET_FROM_EXTRUDER -1   //@20C
  //define Z_PROBE_OFFSET_FROM_EXTRUDER -0.7  //@120C
  //#define Z_PROBE_OFFSET_FROM_EXTRUDER -0.875  //@65C
  

  #define Z_RAISE_BEFORE_HOMING 4       // (in mm) Raise Z axis before homing (G28) for Z probe clearance.
                                        // Be sure you have this distance over your Z_MAX_POS in case.

  #define XY_TRAVEL_SPEED 14400         // X and Y axis travel speed between probes, in mm/min.

  #define Z_RAISE_BEFORE_PROBING 20   // How much the Z axis will be raised before traveling to the first probing point.
  #define Z_RAISE_BETWEEN_PROBINGS 3  // How much the Z axis will be raised when traveling from between next probing points.
  #define Z_RAISE_AFTER_PROBING 20    // How much the Z axis will be raised after the last probing point.

  //#define Z_PROBE_END_SCRIPT "G1 Z10 F12000\nG1 X15 Y330\nG1 Z0.5\nG1 Z10" // These commands will be executed in the end of G29 routine.
                                                                             // Useful to retract a deployable Z probe.
  #define Z_PROBE_END_SCRIPT "G0 X200 Y150 F21600"

  //#define Z_PROBE_SLED // Turn on if you have a Z probe mounted on a sled like those designed by Charles Bell.
  //#define SLED_DOCKING_OFFSET 5 // The extra distance the X axis must travel to pickup the sled. 0 should be fine but you can push it further if you'd like.


  //If you have enabled the Bed Auto Leveling and are using the same Z Probe for Z Homing,
  //it is highly recommended you let this Z_SAFE_HOMING enabled!!!

  #define Z_SAFE_HOMING   // This feature is meant to avoid Z homing with Z probe outside the bed area.
                          // When defined, it will:
                          // - Allow Z homing only after X and Y homing AND stepper drivers still enabled.
                          // - If stepper drivers timeout, it will need X and Y homing again before Z homing.
                          // - Position the Z probe in a defined XY point before Z Homing when homing all axis (G28).
                          // - Block Z homing only when the Z probe is outside bed area.

  #if ENABLED(Z_SAFE_HOMING)

    #define Z_SAFE_HOMING_X_POINT ((X_MIN_POS + X_MAX_POS) / 2)    // X point for Z homing when homing all axis (G28).
    #define Z_SAFE_HOMING_Y_POINT ((Y_MIN_POS + Y_MAX_POS) / 2)    // Y point for Z homing when homing all axis (G28).

  #endif

  // Support for a dedicated Z probe endstop separate from the Z min endstop.
  // If you would like to use both a Z probe and a Z min endstop together,
  // uncomment #define Z_MIN_PROBE_ENDSTOP and read the instructions below.
  // If you still want to use the Z min endstop for homing, disable Z_SAFE_HOMING above.
  // Example: To park the head outside the bed area when homing with G28.
  //
  // WARNING:
  // The Z min endstop will need to set properly as it would without a Z probe
  // to prevent head crashes and premature stopping during a print.
  //
  // To use a separate Z probe endstop, you must have a Z_MIN_PROBE_PIN
  // defined in the pins_XXXXX.h file for your control board.
  // If you are using a servo based Z probe, you will need to enable NUM_SERVOS,
  // Z_ENDSTOP_SERVO_NR and SERVO_ENDSTOP_ANGLES in the R/C SERVO support below.
  // RAMPS 1.3/1.4 boards may be able to use the 5V, Ground and the D32 pin
  // in the Aux 4 section of the RAMPS board. Use 5V for powered sensors,
  // otherwise connect to ground and D32 for normally closed configuration
  // and 5V and D32 for normally open configurations.
  // Normally closed configuration is advised and assumed.
  // The D32 pin in Aux 4 on RAMPS maps to the Arduino D32 pin.
  // Z_MIN_PROBE_PIN is setting the pin to use on the Arduino.
  // Since the D32 pin on the RAMPS maps to D32 on Arduino, this works.
  // D32 is currently selected in the RAMPS 1.3/1.4 pin file.
  // All other boards will need changes to the respective pins_XXXXX.h file.
  //
  // WARNING:
  // Setting the wrong pin may have unexpected and potentially disastrous outcomes.
  // Use with caution and do your homework.
  //
  //#define Z_MIN_PROBE_ENDSTOP

#endif // AUTO_BED_LEVELING_FEATURE







//=============================================================================
//============================= Additional Features ===========================
//=============================================================================

// Custom M code points
#define CUSTOM_M_CODES
#if ENABLED(CUSTOM_M_CODES)
  #if ENABLED(AUTO_BED_LEVELING_FEATURE)
    #define CUSTOM_M_CODE_SET_Z_PROBE_OFFSET 851
    #define Z_PROBE_OFFSET_RANGE_MIN -20
    #define Z_PROBE_OFFSET_RANGE_MAX 20
  #endif
#endif

// EEPROM
// The microcontroller can store settings in the EEPROM, e.g. max velocity...
// M500 - stores parameters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
//define this to enable EEPROM support
#define EEPROM_SETTINGS
//to disable EEPROM Serial responses and decrease program space by ~1700 byte: comment this out:
// please keep turned on if you can.
#define EEPROM_CHITCHAT

// Preheat Constants
#define PLA_PREHEAT_HOTEND_TEMP 180
#define PLA_PREHEAT_HPB_TEMP 50
#define PLA_PREHEAT_FAN_SPEED 0   // Insert Value between 0 and 255

#define ABS_PREHEAT_HOTEND_TEMP 230
#define ABS_PREHEAT_HPB_TEMP 85
#define ABS_PREHEAT_FAN_SPEED 0   // Insert Value between 0 and 255







/**********************************************************************\
 * Support for a filament diameter sensor
 **********************************************************************/
// Uncomment below to enable
//#define FILAMENT_SENSOR

#define FILAMENT_SENSOR_EXTRUDER_NUM 0   //The number of the extruder that has the filament sensor (0,1,2)
#define MEASUREMENT_DELAY_CM        14   //measurement delay in cm.  This is the distance from filament sensor to middle of barrel

#define DEFAULT_NOMINAL_FILAMENT_DIA 3.00  //Enter the diameter (in mm) of the filament generally used (3.0 mm or 1.75 mm) - this is then used in the slicer software.  Used for sensor reading validation
#define MEASURED_UPPER_LIMIT         3.30  //upper limit factor used for sensor reading validation in mm
#define MEASURED_LOWER_LIMIT         1.90  //lower limit factor for sensor reading validation in mm
#define MAX_MEASUREMENT_DELAY       20     //delay buffer size in bytes (1 byte = 1cm)- limits maximum measurement delay allowable (must be larger than MEASUREMENT_DELAY_CM  and lower number saves RAM)

//defines used in the code
#define DEFAULT_MEASURED_FILAMENT_DIA  DEFAULT_NOMINAL_FILAMENT_DIA  //set measured to nominal initially

//When using an LCD, uncomment the line below to display the Filament sensor data on the last line instead of status.  Status will appear for 5 sec.
//#define FILAMENT_LCD_DISPLAY




//==============================LCD and SD support=============================

#define SDSUPPORT // Enable SD Card Support in Hardware Console

#define LCD_TIMEOUT_TO_STATUS 300000

#define DISPLAY_CHARSET_HD44780_JAPAN     // "ääööüüß23°"

#define REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER

#if defined (REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
 #define DOGLCD
 #define U8GLIB_ST7920
 #define REPRAP_DISCOUNT_SMART_CONTROLLER
#endif

// default LCD contrast for dogm-like LCD displays
#ifdef DOGLCD
# ifndef DEFAULT_LCD_CONTRAST
#  define DEFAULT_LCD_CONTRAST 32
# endif
#endif

// Increase the FAN pwm frequency. Removes the PWM noise but increases heating in the FET/Arduino
#define FAST_PWM_FAN

//#define SOFT_PWM_SCALE 0













//#include "configurations/transitional_default_configurations/default/Configuration_adv.h"
#include "Configuration_adv.h"
#include "thermistortables.h"

#endif //CONFIGURATION_H













/* References
 * http://reprap.org/wiki/PID_Tuning
 * http://prusaprinters.org/calculator/
 * 
*/


  
