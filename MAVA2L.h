#pragma once
#ifndef MAVA2L_H
#define MAVA2L_H


#define MAVLINK_XCP_PARAMETERS 717
#include<string>

using namespace std;
string param_c[MAVLINK_XCP_PARAMETERS] = {
   "SYSID_SW_MREV"  , // Eeprom format version number, This value is incremented when changes are made to the eeprom format
   "SYSID_SW_TYPE"  , // Software Type, This is used by the ground station to recognise the software type (eg ArduPlane vs ArduCopter)
   "SYSID_THISMAV"  , // MAVLink system ID of this vehicle, Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network
   "SYSID_MYGCS"    , // My ground station number, Allows restricting radio overrides to only come from my ground station
   "CLI_ENABLED"    , // CLI Enable, This enables/disables the checking for three carriage returns on telemetry links on startup to enter the diagnostics command line interface
   "PILOT_THR_FILT" , // Throttle filter cutoff, Throttle filter cutoff (Hz) - active whenever altitude control is inactive - 0 to disable
   "PILOT_TKOFF_ALT", // Pilot takeoff altitude, Altitude that altitude control modes will climb to when a takeoff is triggered with the throttle stick.
   "PILOT_TKOFF_DZ" , // Takeoff trigger deadzone, Offset from mid stick at which takeoff is triggered
   "PILOT_THR_BHV"  , // Throttle stick behavior, Bitmask containing various throttle stick options. Add up the values for options that you want.
   "TELEM_DELAY"    , // Telemetry startup delay, The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
   "GCS_PID_MASK"   , // GCS PID tuning mask, bitmask of PIDs to send MAVLink PID_TUNING messages for
   "RTL_ALT"        , // RTL Altitude, The minimum relative altitude the model will move to before Returning to Launch.  Set to zero to return at current altitude.
   "RTL_CONE_SLOPE" , // RTL cone slope, Defines a cone above home which determines maximum climb
   "RTL_SPEED"      , // RTL speed, Defines the speed in cm/s which the aircraft will attempt to maintain horizontally while flying home. If this is set to zero, WPNAV_SPEED will be used instead.
   "RNGFND_GAIN"    , // Rangefinder gain, Used to adjust the speed with which the target altitude is changed when objects are sensed below the copter
   "FS_BATT_ENABLE" , // Battery Failsafe Enable, Controls whether failsafe will be invoked when battery voltage or current runs low
   "FS_BATT_VOLTAGE", // Failsafe battery voltage, Battery voltage to trigger failsafe. Set to 0 to disable battery voltage failsafe. If the battery voltage drops below this voltage then the copter will RTL
   "FS_BATT_MAH"    , // Failsafe battery milliAmpHours, Battery capacity remaining to trigger failsafe. Set to 0 to disable battery remaining failsafe. If the battery remaining drops below this level then the copter will RTL
   "FS_GCS_ENABLE"  , // Ground Station Failsafe Enable, Controls whether failsafe will be invoked (and what action to take) when connection with Ground station is lost for at least 5 seconds. NB. The GCS Failsafe is only active when RC_OVERRIDE is being used to control the vehicle.
   "GPS_HDOP_GOOD"  , // GPS Hdop Good, GPS Hdop value at or below this value represent a good position.  Used for pre-arm checks
   "MAG_ENABLE"     , // Compass enable/disable, Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass
   "SUPER_SIMPLE"   , // Super Simple Mode, Bitmask to enable Super Simple mode for some flight modes. Setting this to Disabled(0) will disable Super Simple Mode
   "RTL_ALT_FINAL"  , // RTL Final Altitude, This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission.  Set to zero to land.
   "RTL_CLIMB_MIN"  , // RTL minimum climb, The vehicle will climb this many cm during the initial climb portion of the RTL
   "WP_YAW_BEHAVIOR", // Yaw behaviour during missions, Determines how the autopilot controls the yaw during missions and RTL
   "RTL_LOIT_TIME"  , // RTL loiter time, Time (in milliseconds) to loiter above home before beginning final descent
   "LAND_SPEED"     , // Land speed, The descent speed for the final stage of landing in cm/s
   "LAND_SPEED_HIGH", // Land speed high, The descent speed for the first stage of landing in cm/s. If this is zero then WPNAV_SPEED_DN is used
   "PILOT_VELZ_MAX" , // Pilot maximum vertical speed, The maximum vertical velocity the pilot may request in cm/s
   "PILOT_ACCEL_Z"  , // Pilot vertical acceleration, The vertical acceleration used when pilot is controlling the altitude
   "FS_THR_ENABLE"  , // Throttle Failsafe Enable, The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
   "FS_THR_VALUE"   , // Throttle Failsafe Value, The PWM level on channel 3 below which throttle sailsafe triggers
   "THR_DZ"         , // Throttle deadzone, The deadzone above and below mid throttle.  Used in AltHold, Loiter, PosHold flight modes
   "FLTMODE1"       , // Flight Mode 1, Flight mode when Channel 5 pwm is <= 1230
   "FLTMODE2"       , // Flight Mode 2, Flight mode when Channel 5 pwm is >1230, <= 1360
   "FLTMODE3"       , // Flight Mode 3, Flight mode when Channel 5 pwm is >1360, <= 1490
   "FLTMODE4"       , // Flight Mode 4, Flight mode when Channel 5 pwm is >1490, <= 1620
   "FLTMODE5"       , // Flight Mode 5, Flight mode when Channel 5 pwm is >1620, <= 1749
   "FLTMODE6"       , // Flight Mode 6, Flight mode when Channel 5 pwm is >=1750
   "SIMPLE"         , // Simple mode bitmask, Bitmask which holds which flight modes use simple heading mode (eg bit 0 = 1 means Flight Mode 0 uses simple mode)
   "LOG_BITMASK"    , // Log bitmask, 4 byte bitmap of log types to enable 
   "ESC_CALIBRATION", // ESC Calibration, Controls whether ArduCopter will enter ESC calibration on the next restart.  Do not adjust this parameter manually.
   "TUNE"           , // Channel 6 Tuning, Controls which parameters (normally PID gains) are being tuned with transmitter's channel 6 knob
   "TUNE_LOW"       , // Tuning minimum, The minimum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
   "TUNE_HIGH"      , // Tuning maximum, The maximum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
   "FRAME"          , // Frame Orientation (+, X or V), Controls motor mixing for multicopters.  Not used for Tri or Traditional Helicopters.
   "CH7_OPT"        , // Channel 7 option, Select which function if performed when CH7 is above 1800 pwm
   "CH8_OPT"        , // Channel 8 option, Select which function if performed when CH8 is above 1800 pwm
   "CH9_OPT"        , // Channel 9 option, Select which function if performed when CH9 is above 1800 pwm
   "CH10_OPT"       , // Channel 10 option, Select which function if performed when CH10 is above 1800 pwm
   "CH11_OPT"       , // Channel 11 option, Select which function if performed when CH11 is above 1800 pwm
   "CH12_OPT"       , // Channel 12 option, Select which function if performed when CH12 is above 1800 pwm
   "ARMING_CHECK"   , // Arming check, Allows enabling or disabling of pre-arming checks of receiver, accelerometer, barometer, compass and GPS
   "DISARM_DELAY"   , // Disarm delay, Delay before automatic disarm in seconds. A value of zero disables auto disarm.
   "ANGLE_MAX"      , // Angle Max, Maximum lean angle in all flight modes 
   "RC_FEEL_RP"     , // RC Feel Roll/Pitch, RC feel for roll/pitch which controls vehicle response to user input with 0 being extremely soft and 100 being crisp
   "PHLD_BRAKE_RATE", // PosHold braking rate, PosHold flight mode's rotation rate during braking in deg/sec
   "PHLD_BRAKE_ANGLE", // PosHold braking angle max, PosHold flight mode's max lean angle during braking in centi-degrees
   "LAND_REPOSITION", // Land repositioning, Enables user input during LAND mode, the landing phase of RTL, and auto mode landings.
   "FS_EKF_ACTION"  , // EKF Failsafe Action, Controls the action that will be taken when an EKF failsafe is invoked
   "FS_EKF_THRESH"  , // EKF failsafe variance threshold, Allows setting the maximum acceptable compass and velocity variance
   "FS_CRASH_CHECK" , // Crash check enable, This enables automatic crash checking. When enabled the motors will disarm if a crash is detected.
   "RC_SPEED"       , // ESC Update Speed, This is the speed in Hertz that your ESCs will receive updates
   "ACRO_RP_P"      , // Acro Roll and Pitch P gain, Converts pilot roll and pitch into a desired rate of rotation in ACRO and SPORT mode.  Higher values mean faster rate of rotation.
   "ACRO_YAW_P"     , // Acro Yaw P gain, Converts pilot yaw input into a desired rate of rotation in ACRO, Stabilize and SPORT modes.  Higher values mean faster rate of rotation.
   "ACRO_BAL_ROLL"  , // Acro Balance Roll, rate at which roll angle returns to level in acro mode.  A higher value causes the vehicle to return to level faster.
   "ACRO_BAL_PITCH" , // Acro Balance Pitch, rate at which pitch angle returns to level in acro mode.  A higher value causes the vehicle to return to level faster.
   "ACRO_TRAINER"   , // Acro Trainer, Type of trainer used in acro mode   
   "ACRO_EXPO"      , // Acro Expo, Acro roll/pitch Expo to allow faster rotation when stick at edges
   "VEL_XY_P"       , // Velocity (horizontal) P gain, Velocity (horizontal) P gain.  Converts the difference between desired velocity to a target acceleration
   "VEL_XY_I"       , // Velocity (horizontal) I gain, Velocity (horizontal) I gain.  Corrects long-term difference in desired velocity to a target acceleration
   "VEL_XY_IMAX"    , // Velocity (horizontal) integrator maximum, Velocity (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output
   "VEL_Z_P"        , // Velocity (vertical) P gain, Velocity (vertical) P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
   "ACCEL_Z_P"      , // Throttle acceleration controller P gain, Throttle acceleration controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
   "ACCEL_Z_I"      , // Throttle acceleration controller I gain, Throttle acceleration controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
   "ACCEL_Z_IMAX"   , // Throttle acceleration controller I gain maximum, Throttle acceleration controller I gain maximum.  Constrains the maximum pwm that the I term will generate
   "ACCEL_Z_D"      , // Throttle acceleration controller D gain, Throttle acceleration controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
   "ACCEL_Z_FILT_HZ", // Throttle acceleration filter, Filter applied to acceleration to reduce noise.  Lower values reduce noise but add delay.
   "POS_Z_P"        , // Position (vertical) controller P gain, Position (vertical) controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
   "POS_XY_P"       , // Position (horizonal) controller P gain, Loiter position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller
   "AUTOTUNE_AXES"  , // Autotune axis bitmask, 1-byte bitmap of axes to autotune
   "AUTOTUNE_AGGR"  , // Autotune aggressiveness, Autotune aggressiveness. Defines the bounce back used to detect size of the D term.
   "AUTOTUNE_MIN_D" , // AutoTune minimum D, Defines the minimum D gain    
   "THROW_MOT_START", // Start motors before throwing is detected, Used by THROW mode. Controls whether motors will run at the speed set by THR_MIN or will be stopped when armed and waiting for the throw.
   "TERRAIN_FOLLOW" , // Terrain Following use control, This enables terrain following for RTL and LAND flight modes. To use this option TERRAIN_ENABLE must be 1 and the GCS must  support sending terrain data to the aircraft.  In RTL the RTL_ALT will be considered a height above the terrain.  In LAND mode the vehicle will slow to LAND_SPEED 10m above terrain (instead of 10m above home).  This parameter does not affect AUTO and Guided which use a per-command flag to determine if the height is above-home, absolute or above-terrain.
   "TKOFF_NAV_ALT"  , // Takeoff navigation altitude, This is the altitude in meters above the takeoff point that attitude changes for navigation can begin
   "THROW_NEXTMODE" , // Throw mode's follow up mode, Vehicle will switch to this mode after the throw is successfully completed.  Default is to stay in throw mode (18)
   "THROW_TYPE"     , // Type of Type, Used by THROW mode. Specifies whether Copter is thrown upward or dropped.
   "GND_EFFECT_COMP", // Ground Effect Compensation Enable/Disable, Ground Effect Compensation Enable/Disable
   "SERIAL0_BAUD"   , // Serial0 baud rate, The baud rate used on the USB console. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
   "SERIAL0_PROTOCOL", // Console protocol selection, Control what protocol to use on the console. 
   "SERIAL1_PROTOCOL", // Telem1 protocol selection, Control what protocol to use on the Telem1 port. Note that the Frsky options require external converter hardware. See the wiki for details.
   "SERIAL1_BAUD"   , // Telem1 Baud Rate, The baud rate used on the Telem1 port. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
   "SERIAL2_PROTOCOL", // Telemetry 2 protocol selection, Control what protocol to use on the Telem2 port. Note that the Frsky options require external converter hardware. See the wiki for details.
   "SERIAL2_BAUD"   , // Telemetry 2 Baud Rate, The baud rate of the Telem2 port. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
   "SERIAL3_PROTOCOL", // Serial 3 (GPS) protocol selection, Control what protocol Serial 3 (GPS) should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.
   "SERIAL3_BAUD"   , // Serial 3 (GPS) Baud Rate, The baud rate used for the Serial 3 (GPS). The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
   "SERIAL4_PROTOCOL", // Serial4 protocol selection, Control what protocol Serial4 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.
   "SERIAL4_BAUD"   , // Serial 4 Baud Rate, The baud rate used for Serial4. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
   "SERIAL5_PROTOCOL", // Serial5 protocol selection, Control what protocol Serial5 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.
   "SERIAL5_BAUD"   , // Serial 5 Baud Rate, The baud rate used for Serial5. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
   "RC1_MIN"        , // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC1_TRIM"       , // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC1_MAX"        , // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC1_REV"        , // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
   "RC1_DZ"         , // RC dead-zone, dead zone around trim or bottom     
   "RC2_MIN"        , // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC2_TRIM"       , // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC2_MAX"        , // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC2_REV"        , // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
   "RC2_DZ"         , // RC dead-zone, dead zone around trim or bottom     
   "RC3_MIN"        , // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC3_TRIM"       , // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC3_MAX"        , // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC3_REV"        , // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
   "RC3_DZ"         , // RC dead-zone, dead zone around trim or bottom     
   "RC4_MIN"        , // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC4_TRIM"       , // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC4_MAX"        , // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC4_REV"        , // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
   "RC4_DZ"         , // RC dead-zone, dead zone around trim or bottom     
   "RC5_MIN"        , // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC5_TRIM"       , // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC5_MAX"        , // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC5_REV"        , // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
   "RC5_DZ"         , // RC dead-zone, dead zone around trim or bottom     
   "RC5_FUNCTION"   , // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
   "RC6_MIN"        , // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC6_TRIM"       , // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC6_MAX"        , // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC6_REV"        , // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
   "RC6_DZ"         , // RC dead-zone, dead zone around trim or bottom     
   "RC6_FUNCTION"   , // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
   "RC7_MIN"        , // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC7_TRIM"       , // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC7_MAX"        , // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC7_REV"        , // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
   "RC7_DZ"         , // RC dead-zone, dead zone around trim or bottom     
   "RC7_FUNCTION"   , // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
   "RC8_MIN"        , // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC8_TRIM"       , // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC8_MAX"        , // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC8_REV"        , // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
   "RC8_DZ"         , // RC dead-zone, dead zone around trim or bottom     
   "RC8_FUNCTION"   , // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
   "RC9_MIN"        , // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC9_TRIM"       , // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC9_MAX"        , // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC9_REV"        , // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
   "RC9_DZ"         , // RC dead-zone, dead zone around trim or bottom     
   "RC9_FUNCTION"   , // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
   "RC10_MIN"       , // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC10_TRIM"      , // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC10_MAX"       , // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC10_REV"       , // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
   "RC10_DZ"        , // RC dead-zone, dead zone around trim or bottom     
   "RC10_FUNCTION"  , // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
   "RC11_MIN"       , // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC11_TRIM"      , // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC11_MAX"       , // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC11_REV"       , // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
   "RC11_DZ"        , // RC dead-zone, dead zone around trim or bottom     
   "RC11_FUNCTION"  , // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
   "RC12_MIN"       , // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC12_TRIM"      , // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC12_MAX"       , // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC12_REV"       , // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
   "RC12_DZ"        , // RC dead-zone, dead zone around trim or bottom     
   "RC12_FUNCTION"  , // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
   "RC13_MIN"       , // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC13_TRIM"      , // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC13_MAX"       , // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC13_REV"       , // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
   "RC13_DZ"        , // RC dead-zone, dead zone around trim or bottom     
   "RC13_FUNCTION"  , // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
   "RC14_MIN"       , // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC14_TRIM"      , // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC14_MAX"       , // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
   "RC14_REV"       , // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
   "RC14_DZ"        , // RC dead-zone, dead zone around trim or bottom     
   "RC14_FUNCTION"  , // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
   "CAM_TRIGG_TYPE" , // Camera shutter (trigger) type, how to trigger the camera to take a picture
   "CAM_DURATION"   , // Duration that shutter is held open, How long the shutter will be held open in 10ths of a second (i.e. enter 10 for 1second, 50 for 5seconds)
   "CAM_SERVO_ON"   , // Servo ON PWM value, PWM value to move servo to when shutter is activated
   "CAM_SERVO_OFF"  , // Servo OFF PWM value, PWM value to move servo to when shutter is deactivated
   "CAM_TRIGG_DIST" , // Camera trigger distance, Distance in meters between camera triggers. If this value is non-zero then the camera will trigger whenever the GPS position changes by this number of meters regardless of what mode the APM is in. Note that this parameter can also be set in an auto mission using the DO_SET_CAM_TRIGG_DIST command, allowing you to enable/disable the triggering of the camera during the flight.
   "CAM_RELAY_ON"   , // Relay ON value, This sets whether the relay goes high or low when it triggers. Note that you should also set RELAY_DEFAULT appropriately for your camera
   "CAM_MIN_INTERVAL", // Minimum time between photos, Postpone shooting if previous picture was taken less than preset time(ms) ago.
   "CAM_MAX_ROLL"   , // Maximum photo roll angle., Postpone shooting if roll is greater than limit. (0=Disable, will shoot regardless of roll).
   "CAM_FEEDBACK_PIN", // Camera feedback pin, pin number to use for save accurate camera feedback messages. If set to -1 then don't use a pin flag for this, otherwise this is a pin number which if held high after a picture trigger order, will save camera messages when camera really takes a picture. A universal camera hot shoe is needed. The pin should be held high for at least 2 milliseconds for reliable trigger detection. See also the CAM_FEEDBACK_POL option. If using AUX4 pin on a Pixhawk then a fast capture method is used that allows for the trigger time to be as short as one microsecond.
   "CAM_FEEDBACK_POL", // Camera feedback pin polarity, Polarity for feedback pin. If this is 1 then the feedback pin should go high on trigger. If set to 0 then it should go low
   "RELAY_PIN"      , // First Relay Pin, Digital pin number for first relay control. This is the pin used for camera control.
   "RELAY_PIN2"     , // Second Relay Pin, Digital pin number for 2nd relay control.
   "RELAY_PIN3"     , // Third Relay Pin, Digital pin number for 3rd relay control.
   "RELAY_PIN4"     , // Fourth Relay Pin, Digital pin number for 4th relay control.
   "RELAY_DEFAULT"  , // Default relay state, The state of the relay on boot. 
   "EPM_ENABLE"     , // EPM Enable/Disable, EPM enable/disable            
   "EPM_GRAB"       , // EPM Grab PWM, PWM value sent to EPM to initiate grabbing the cargo
   "EPM_RELEASE"    , // EPM Release PWM, PWM value sent to EPM to release the cargo
   "EPM_NEUTRAL"    , // EPM Neutral PWM, PWM value sent to EPM when not grabbing or releasing
   "EPM_REGRAB"     , // EPM UAVCAN Hardpoint ID, Refer to https://docs.zubax.com/opengrab_epm_v3#UAVCAN_interface
   "CHUTE_ENABLED"  , // Parachute release enabled or disabled, Parachute release enabled or disabled
   "CHUTE_TYPE"     , // Parachute release mechanism type (relay or servo), Parachute release mechanism type (relay or servo)
   "CHUTE_SERVO_ON" , // Parachute Servo ON PWM value, Parachute Servo PWM value when parachute is released
   "CHUTE_SERVO_OFF", // Servo OFF PWM value, Parachute Servo PWM value when parachute is not released
   "CHUTE_ALT_MIN"  , // Parachute min altitude in meters above home, Parachute min altitude above home.  Parachute will not be released below this altitude.  0 to disable alt check.
   "CHUTE_DELAY_MS" , // Parachute release delay, Delay in millseconds between motor stop and chute release
   "LGR_SERVO_RTRACT", // Landing Gear Servo Retracted PWM Value, Servo PWM value when landing gear is retracted
   "LGR_SERVO_DEPLOY", // Landing Gear Servo Deployed PWM Value, Servo PWM value when landing gear is deployed
   "IM_STAB_COL_1"  , // Stabilize Mode Collective Point 1, Helicopter's minimum collective pitch setting at zero throttle input in Stabilize mode
   "IM_STAB_COL_2"  , // Stabilize Mode Collective Point 2, Helicopter's collective pitch setting at mid-low throttle input in Stabilize mode
   "IM_STAB_COL_3"  , // Stabilize Mode Collective Point 3, Helicopter's collective pitch setting at mid-high throttle input in Stabilize mode
   "IM_STAB_COL_4"  , // Stabilize Mode Collective Point 4, Helicopter's maximum collective pitch setting at full throttle input in Stabilize mode
   "IM_ACRO_COL_EXP", // Acro Mode Collective Expo, Used to soften collective pitch inputs near center point in Acro mode.
   "COMPASS_OFS_X"  , // Compass offsets in milligauss on the X axis, Offset to be added to the compass x-axis values to compensate for metal in the frame
   "COMPASS_OFS_Y"  , // Compass offsets in milligauss on the Y axis, Offset to be added to the compass y-axis values to compensate for metal in the frame
   "COMPASS_OFS_Z"  , // Compass offsets in milligauss on the Z axis, Offset to be added to the compass z-axis values to compensate for metal in the frame
   "COMPASS_DEC"    , // Compass declination, An angle to compensate between the true north and magnetic north
   "COMPASS_LEARN"  , // Learn compass offsets automatically, Enable or disable the automatic learning of compass offsets. You can enable learning either using a compass-only method that is suitable only for fixed wing aircraft or using the offsets learnt by the active EKF state estimator. If this option is enabled then the learnt offsets are saved when you disarm the vehicle.
   "COMPASS_USE"    , // Use compass for yaw, Enable or disable the use of the compass (instead of the GPS) for determining heading
   "COMPASS_AUTODEC", // Auto Declination, Enable or disable the automatic calculation of the declination based on gps location
   "COMPASS_MOTCT"  , // Motor interference compensation type, Set motor interference compensation type to disabled, throttle or current.  Do not change manually.
   "COMPASS_MOT_X"  , // Motor interference compensation for body frame X axis, Multiplied by the current throttle and added to the compass's x-axis values to compensate for motor interference
   "COMPASS_MOT_Y"  , // Motor interference compensation for body frame Y axis, Multiplied by the current throttle and added to the compass's y-axis values to compensate for motor interference
   "COMPASS_MOT_Z"  , // Motor interference compensation for body frame Z axis, Multiplied by the current throttle and added to the compass's z-axis values to compensate for motor interference
   "COMPASS_ORIENT" , // Compass orientation, The orientation of the compass relative to the autopilot board. This will default to the right value for each board type, but can be changed if you have an external compass. See the documentation for your external compass for the right value. The correct orientation should give the X axis forward, the Y axis to the right and the Z axis down. So if your aircraft is pointing west it should show a positive value for the Y axis, and a value close to zero for the X axis. On a PX4 or Pixhawk with an external compass the correct value is zero if the compass is correctly oriented. NOTE: This orientation is combined with any AHRS_ORIENTATION setting.
   "COMPASS_EXTERNAL", // Compass is attached via an external cable, Configure compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. Set to 1 if the compass is externally connected. When externally connected the COMPASS_ORIENT option operates independently of the AHRS_ORIENTATION board orientation option. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
   "COMPASS_OFS2_X" , // Compass2 offsets in milligauss on the X axis, Offset to be added to compass2's x-axis values to compensate for metal in the frame
   "COMPASS_OFS2_Y" , // Compass2 offsets in milligauss on the Y axis, Offset to be added to compass2's y-axis values to compensate for metal in the frame
   "COMPASS_OFS2_Z" , // Compass2 offsets in milligauss on the Z axis, Offset to be added to compass2's z-axis values to compensate for metal in the frame
   "COMPASS_MOT2_X" , // Motor interference compensation to compass2 for body frame X axis, Multiplied by the current throttle and added to compass2's x-axis values to compensate for motor interference
   "COMPASS_MOT2_Y" , // Motor interference compensation to compass2 for body frame Y axis, Multiplied by the current throttle and added to compass2's y-axis values to compensate for motor interference
   "COMPASS_MOT2_Z" , // Motor interference compensation to compass2 for body frame Z axis, Multiplied by the current throttle and added to compass2's z-axis values to compensate for motor interference
   "COMPASS_PRIMARY", // Choose primary compass, If more than one compass is available this selects which compass is the primary. Normally 0=External, 1=Internal. If no External compass is attached this parameter is ignored
   "COMPASS_OFS3_X" , // Compass3 offsets in milligauss on the X axis, Offset to be added to compass3's x-axis values to compensate for metal in the frame
   "COMPASS_OFS3_Y" , // Compass3 offsets in milligauss on the Y axis, Offset to be added to compass3's y-axis values to compensate for metal in the frame
   "COMPASS_OFS3_Z" , // Compass3 offsets in milligauss on the Z axis, Offset to be added to compass3's z-axis values to compensate for metal in the frame
   "COMPASS_MOT3_X" , // Motor interference compensation to compass3 for body frame X axis, Multiplied by the current throttle and added to compass3's x-axis values to compensate for motor interference
   "COMPASS_MOT3_Y" , // Motor interference compensation to compass3 for body frame Y axis, Multiplied by the current throttle and added to compass3's y-axis values to compensate for motor interference
   "COMPASS_MOT3_Z" , // Motor interference compensation to compass3 for body frame Z axis, Multiplied by the current throttle and added to compass3's z-axis values to compensate for motor interference
   "COMPASS_DEV_ID" , // Compass device id, Compass device id.  Automatically detected, do not set manually
   "COMPASS_DEV_ID2", // Compass2 device id, Second compass's device id.  Automatically detected, do not set manually
   "COMPASS_DEV_ID3", // Compass3 device id, Third compass's device id.  Automatically detected, do not set manually
   "COMPASS_USE2"   , // Compass2 used for yaw, Enable or disable the second compass for determining heading.
   "COMPASS_ORIENT2", // Compass2 orientation, The orientation of the second compass relative to the frame (if external) or autopilot board (if internal).
   "COMPASS_EXTERN2", // Compass2 is attached via an external cable, Configure second compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
   "COMPASS_USE3"   , // Compass3 used for yaw, Enable or disable the third compass for determining heading.
   "COMPASS_ORIENT3", // Compass3 orientation, The orientation of the third compass relative to the frame (if external) or autopilot board (if internal).
   "COMPASS_EXTERN3", // Compass3 is attached via an external cable, Configure third compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
   "COMPASS_DIA_X"  , // Compass soft-iron diagonal X component, DIA_X in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
   "COMPASS_DIA_Y"  , // Compass soft-iron diagonal Y component, DIA_Y in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
   "COMPASS_DIA_Z"  , // Compass soft-iron diagonal Z component, DIA_Z in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
   "COMPASS_ODI_X"  , // Compass soft-iron off-diagonal X component, ODI_X in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
   "COMPASS_ODI_Y"  , // Compass soft-iron off-diagonal Y component, ODI_Y in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
   "COMPASS_ODI_Z"  , // Compass soft-iron off-diagonal Z component, ODI_Z in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
   "COMPASS_DIA2_X" , // Compass2 soft-iron diagonal X component, DIA_X in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
   "COMPASS_DIA2_Y" , // Compass2 soft-iron diagonal Y component, DIA_Y in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
   "COMPASS_DIA2_Z" , // Compass2 soft-iron diagonal Z component, DIA_Z in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
   "COMPASS_ODI2_X" , // Compass2 soft-iron off-diagonal X component, ODI_X in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
   "COMPASS_ODI2_Y" , // Compass2 soft-iron off-diagonal Y component, ODI_Y in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
   "COMPASS_ODI2_Z" , // Compass2 soft-iron off-diagonal Z component, ODI_Z in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
   "COMPASS_DIA3_X" , // Compass3 soft-iron diagonal X component, DIA_X in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
   "COMPASS_DIA3_Y" , // Compass3 soft-iron diagonal Y component, DIA_Y in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
   "COMPASS_DIA3_Z" , // Compass3 soft-iron diagonal Z component, DIA_Z in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
   "COMPASS_ODI3_X" , // Compass3 soft-iron off-diagonal X component, ODI_X in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
   "COMPASS_ODI3_Y" , // Compass3 soft-iron off-diagonal Y component, ODI_Y in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
   "COMPASS_ODI3_Z" , // Compass3 soft-iron off-diagonal Z component, ODI_Z in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
   "COMPASS_CAL_FIT", // Compass calibration fitness, This controls the fitness level required for a successful compass calibration. A lower value makes for a stricter fit (less likely to pass). This is the value used for the primary magnetometer. Other magnetometers get double the value.
   "INS_PRODUCT_ID" , // IMU Product ID, Which type of IMU is installed (read-only).
   "INS_GYROFFS_X"  , // Gyro offsets of X axis, Gyro sensor offsets of X axis. This is setup on each boot during gyro calibrations
   "INS_GYROFFS_Y"  , // Gyro offsets of Y axis, Gyro sensor offsets of Y axis. This is setup on each boot during gyro calibrations
   "INS_GYROFFS_Z"  , // Gyro offsets of Z axis, Gyro sensor offsets of Z axis. This is setup on each boot during gyro calibrations
   "INS_GYR2OFFS_X" , // Gyro2 offsets of X axis, Gyro2 sensor offsets of X axis. This is setup on each boot during gyro calibrations
   "INS_GYR2OFFS_Y" , // Gyro2 offsets of Y axis, Gyro2 sensor offsets of Y axis. This is setup on each boot during gyro calibrations
   "INS_GYR2OFFS_Z" , // Gyro2 offsets of Z axis, Gyro2 sensor offsets of Z axis. This is setup on each boot during gyro calibrations
   "INS_GYR3OFFS_X" , // Gyro3 offsets of X axis, Gyro3 sensor offsets of X axis. This is setup on each boot during gyro calibrations
   "INS_GYR3OFFS_Y" , // Gyro3 offsets of Y axis, Gyro3 sensor offsets of Y axis. This is setup on each boot during gyro calibrations
   "INS_GYR3OFFS_Z" , // Gyro3 offsets of Z axis, Gyro3 sensor offsets of Z axis. This is setup on each boot during gyro calibrations
   "INS_ACCSCAL_X"  , // Accelerometer scaling of X axis, Accelerometer scaling of X axis.  Calculated during acceleration calibration routine
   "INS_ACCSCAL_Y"  , // Accelerometer scaling of Y axis, Accelerometer scaling of Y axis  Calculated during acceleration calibration routine
   "INS_ACCSCAL_Z"  , // Accelerometer scaling of Z axis, Accelerometer scaling of Z axis  Calculated during acceleration calibration routine
   "INS_ACCOFFS_X"  , // Accelerometer offsets of X axis, Accelerometer offsets of X axis. This is setup using the acceleration calibration or level operations
   "INS_ACCOFFS_Y"  , // Accelerometer offsets of Y axis, Accelerometer offsets of Y axis. This is setup using the acceleration calibration or level operations
   "INS_ACCOFFS_Z"  , // Accelerometer offsets of Z axis, Accelerometer offsets of Z axis. This is setup using the acceleration calibration or level operations
   "INS_ACC2SCAL_X" , // Accelerometer2 scaling of X axis, Accelerometer2 scaling of X axis.  Calculated during acceleration calibration routine
   "INS_ACC2SCAL_Y" , // Accelerometer2 scaling of Y axis, Accelerometer2 scaling of Y axis  Calculated during acceleration calibration routine
   "INS_ACC2SCAL_Z" , // Accelerometer2 scaling of Z axis, Accelerometer2 scaling of Z axis  Calculated during acceleration calibration routine
   "INS_ACC2OFFS_X" , // Accelerometer2 offsets of X axis, Accelerometer2 offsets of X axis. This is setup using the acceleration calibration or level operations
   "INS_ACC2OFFS_Y" , // Accelerometer2 offsets of Y axis, Accelerometer2 offsets of Y axis. This is setup using the acceleration calibration or level operations
   "INS_ACC2OFFS_Z" , // Accelerometer2 offsets of Z axis, Accelerometer2 offsets of Z axis. This is setup using the acceleration calibration or level operations
   "INS_ACC3SCAL_X" , // Accelerometer3 scaling of X axis, Accelerometer3 scaling of X axis.  Calculated during acceleration calibration routine
   "INS_ACC3SCAL_Y" , // Accelerometer3 scaling of Y axis, Accelerometer3 scaling of Y axis  Calculated during acceleration calibration routine
   "INS_ACC3SCAL_Z" , // Accelerometer3 scaling of Z axis, Accelerometer3 scaling of Z axis  Calculated during acceleration calibration routine
   "INS_ACC3OFFS_X" , // Accelerometer3 offsets of X axis, Accelerometer3 offsets of X axis. This is setup using the acceleration calibration or level operations
   "INS_ACC3OFFS_Y" , // Accelerometer3 offsets of Y axis, Accelerometer3 offsets of Y axis. This is setup using the acceleration calibration or level operations
   "INS_ACC3OFFS_Z" , // Accelerometer3 offsets of Z axis, Accelerometer3 offsets of Z axis. This is setup using the acceleration calibration or level operations
   "INS_GYRO_FILTER", // Gyro filter cutoff frequency, Filter cutoff frequency for gyroscopes. This can be set to a lower value to try to cope with very high vibration levels in aircraft. This option takes effect on the next reboot. A value of zero means no filtering (not recommended!)
   "INS_ACCEL_FILTER", // Accel filter cutoff frequency, Filter cutoff frequency for accelerometers. This can be set to a lower value to try to cope with very high vibration levels in aircraft. This option takes effect on the next reboot. A value of zero means no filtering (not recommended!)
   "INS_USE"        , // Use first IMU for attitude, velocity and position estimates, Use first IMU for attitude, velocity and position estimates
   "INS_USE2"       , // Use second IMU for attitude, velocity and position estimates, Use second IMU for attitude, velocity and position estimates
   "INS_USE3"       , // Use third IMU for attitude, velocity and position estimates, Use third IMU for attitude, velocity and position estimates
   "INS_STILL_THRESH", // Stillness threshold for detecting if we are moving, Threshold to tolerate vibration to determine if vehicle is motionless. This depends on the frame type and if there is a constant vibration due to motors before launch or after landing. Total motionless is about 0.05. Suggested values: Planes/rover use 0.1, multirotors use 1, tradHeli uses 5
   "INS_GYR_CAL"    , // Gyro Calibration scheme, Conrols when automatic gyro calibration is performed
   "INS_TRIM_OPTION", // Accel cal trim option, Specifies how the accel cal routine determines the trims
   "INS_ACC_BODYFIX", // Body-fixed accelerometer, The body-fixed accelerometer to be used for trim calculation
   "WPNAV_SPEED"    , // Waypoint Horizontal Speed Target, Defines the speed in cm/s which the aircraft will attempt to maintain horizontally during a WP mission
   "WPNAV_RADIUS"   , // Waypoint Radius, Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
   "WPNAV_SPEED_UP" , // Waypoint Climb Speed Target, Defines the speed in cm/s which the aircraft will attempt to maintain while climbing during a WP mission
   "WPNAV_SPEED_DN" , // Waypoint Descent Speed Target, Defines the speed in cm/s which the aircraft will attempt to maintain while descending during a WP mission
   "WPNAV_LOIT_SPEED", // Loiter Horizontal Maximum Speed, Defines the maximum speed in cm/s which the aircraft will travel horizontally while in loiter mode
   "WPNAV_ACCEL"    , // Waypoint Acceleration , Defines the horizontal acceleration in cm/s/s used during missions
   "WPNAV_ACCEL_Z"  , // Waypoint Vertical Acceleration, Defines the vertical acceleration in cm/s/s used during missions
   "WPNAV_LOIT_JERK", // Loiter maximum jerk, Loiter maximum jerk in cm/s/s/s
   "WPNAV_LOIT_MAXA", // Loiter maximum acceleration, Loiter maximum acceleration in cm/s/s.  Higher values cause the copter to accelerate and stop more quickly.
   "WPNAV_LOIT_MINA", // Loiter minimum acceleration, Loiter minimum acceleration in cm/s/s. Higher values stop the copter more quickly when the stick is centered, but cause a larger jerk when the copter stops.
   "CIRCLE_RADIUS"  , // Circle Radius, Defines the radius of the circle the vehicle will fly when in Circle flight mode
   "CIRCLE_RATE"    , // Circle rate, Circle mode's turn rate in deg/sec.  Positive to turn clockwise, negative for counter clockwise
   "ATC_SLEW_YAW"   , // Yaw target slew rate, Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes
   "ATC_ACCEL_Y_MAX", // Acceleration Max for Yaw, Maximum acceleration in yaw axis
   "ATC_RATE_FF_ENAB", // Rate Feedforward Enable, Controls whether body-frame rate feedfoward is enabled or disabled
   "ATC_ACCEL_R_MAX", // Acceleration Max for Roll, Maximum acceleration in roll axis
   "ATC_ACCEL_P_MAX", // Acceleration Max for Pitch, Maximum acceleration in pitch axis
   "ATC_ANGLE_BOOST", // Angle Boost, Angle Boost increases output throttle as the vehicle leans to reduce loss of altitude
   "ATC_ANG_RLL_P"  , // Roll axis angle controller P gain, Roll axis angle controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate
   "ATC_ANG_PIT_P"  , // Pitch axis angle controller P gain, Pitch axis angle controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
   "ATC_ANG_YAW_P"  , // Yaw axis angle controller P gain, Yaw axis angle controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate
   "ATC_ANG_LIM_TC" , // Angle Limit (to maintain altitude) Time Constant, Angle Limit (to maintain altitude) Time Constant
   "ATC_RAT_RLL_P"  , // Roll axis rate controller P gain, Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
   "ATC_RAT_RLL_I"  , // Roll axis rate controller I gain, Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
   "ATC_RAT_RLL_IMAX", // Roll axis rate controller I gain maximum, Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
   "ATC_RAT_RLL_D"  , // Roll axis rate controller D gain, Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
   "ATC_RAT_RLL_FILT", // Roll axis rate conroller input frequency in Hz, Roll axis rate conroller input frequency in Hz
   "ATC_RAT_PIT_P"  , // Pitch axis rate controller P gain, Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output
   "ATC_RAT_PIT_I"  , // Pitch axis rate controller I gain, Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
   "ATC_RAT_PIT_IMAX", // Pitch axis rate controller I gain maximum, Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
   "ATC_RAT_PIT_D"  , // Pitch axis rate controller D gain, Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
   "ATC_RAT_PIT_FILT", // Pitch axis rate conroller input frequency in Hz, Pitch axis rate conroller input frequency in Hz
   "ATC_RAT_YAW_P"  , // Yaw axis rate controller P gain, Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
   "ATC_RAT_YAW_I"  , // Yaw axis rate controller I gain, Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
   "ATC_RAT_YAW_IMAX", // Yaw axis rate controller I gain maximum, Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
   "ATC_RAT_YAW_D"  , // Yaw axis rate controller D gain, Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
   "ATC_RAT_YAW_FILT", // Yaw axis rate conroller input frequency in Hz, Yaw axis rate conroller input frequency in Hz
   "ATC_THR_MIX_MIN", // Throttle Mix Minimum, Throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
   "ATC_THR_MIX_MAX", // Throttle Mix Maximum, Throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
   "POSCON__ACC_XY_FILT", // XY Acceleration filter cutoff frequency, Lower values will slow the response of the navigation controller and reduce twitchiness
   "SR0_RAW_SENS"   , // Raw sensor stream rate, Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
   "SR0_EXT_STAT"   , // Extended status stream rate to ground station, Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
   "SR0_RC_CHAN"    , // RC Channel stream rate to ground station, Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS_RAW to ground station
   "SR0_RAW_CTRL"   , // Raw Control stream rate to ground station, Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station
   "SR0_POSITION"   , // Position stream rate to ground station, Stream rate of GLOBAL_POSITION_INT to ground station
   "SR0_EXTRA1"     , // Extra data type 1 stream rate to ground station, Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station
   "SR0_EXTRA2"     , // Extra data type 2 stream rate to ground station, Stream rate of VFR_HUD to ground station
   "SR0_EXTRA3"     , // Extra data type 3 stream rate to ground station, Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
   "SR0_PARAMS"     , // Parameter stream rate to ground station, Stream rate of PARAM_VALUE to ground station
   "SR0_ADSB"       , // ADSB stream rate to ground station, ADSB stream rate to ground station
   "SR1_RAW_SENS"   , // Raw sensor stream rate, Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
   "SR1_EXT_STAT"   , // Extended status stream rate to ground station, Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
   "SR1_RC_CHAN"    , // RC Channel stream rate to ground station, Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS_RAW to ground station
   "SR1_RAW_CTRL"   , // Raw Control stream rate to ground station, Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station
   "SR1_POSITION"   , // Position stream rate to ground station, Stream rate of GLOBAL_POSITION_INT to ground station
   "SR1_EXTRA1"     , // Extra data type 1 stream rate to ground station, Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station
   "SR1_EXTRA2"     , // Extra data type 2 stream rate to ground station, Stream rate of VFR_HUD to ground station
   "SR1_EXTRA3"     , // Extra data type 3 stream rate to ground station, Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
   "SR1_PARAMS"     , // Parameter stream rate to ground station, Stream rate of PARAM_VALUE to ground station
   "SR1_ADSB"       , // ADSB stream rate to ground station, ADSB stream rate to ground station
   "SR2_RAW_SENS"   , // Raw sensor stream rate, Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
   "SR2_EXT_STAT"   , // Extended status stream rate to ground station, Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
   "SR2_RC_CHAN"    , // RC Channel stream rate to ground station, Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS_RAW to ground station
   "SR2_RAW_CTRL"   , // Raw Control stream rate to ground station, Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station
   "SR2_POSITION"   , // Position stream rate to ground station, Stream rate of GLOBAL_POSITION_INT to ground station
   "SR2_EXTRA1"     , // Extra data type 1 stream rate to ground station, Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station
   "SR2_EXTRA2"     , // Extra data type 2 stream rate to ground station, Stream rate of VFR_HUD to ground station
   "SR2_EXTRA3"     , // Extra data type 3 stream rate to ground station, Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
   "SR2_PARAMS"     , // Parameter stream rate to ground station, Stream rate of PARAM_VALUE to ground station
   "SR2_ADSB"       , // ADSB stream rate to ground station, ADSB stream rate to ground station
   "SR3_RAW_SENS"   , // Raw sensor stream rate, Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
   "SR3_EXT_STAT"   , // Extended status stream rate to ground station, Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
   "SR3_RC_CHAN"    , // RC Channel stream rate to ground station, Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS_RAW to ground station
   "SR3_RAW_CTRL"   , // Raw Control stream rate to ground station, Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station
   "SR3_POSITION"   , // Position stream rate to ground station, Stream rate of GLOBAL_POSITION_INT to ground station
   "SR3_EXTRA1"     , // Extra data type 1 stream rate to ground station, Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station
   "SR3_EXTRA2"     , // Extra data type 2 stream rate to ground station, Stream rate of VFR_HUD to ground station
   "SR3_EXTRA3"     , // Extra data type 3 stream rate to ground station, Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
   "SR3_PARAMS"     , // Parameter stream rate to ground station, Stream rate of PARAM_VALUE to ground station
   "SR3_ADSB"       , // ADSB stream rate to ground station, ADSB stream rate to ground station
   "AHRS_GPS_GAIN"  , // AHRS GPS gain, This controls how how much to use the GPS to correct the attitude. This should never be set to zero for a plane as it would result in the plane losing control in turns. For a plane please use the default value of 1.0.
   "AHRS_GPS_USE"   , // AHRS use GPS for navigation, This controls whether to use dead-reckoning or GPS based navigation. If set to 0 then the GPS won't be used for navigation, and only dead reckoning will be used. A value of zero should never be used for normal flight.
   "AHRS_YAW_P"     , // Yaw P, This controls the weight the compass or GPS has on the heading. A higher value means the heading will track the yaw source (GPS or compass) more rapidly.
   "AHRS_RP_P"      , // AHRS RP_P, This controls how fast the accelerometers correct the attitude
   "AHRS_WIND_MAX"  , // Maximum wind, This sets the maximum allowable difference between ground speed and airspeed. This allows the plane to cope with a failing airspeed sensor. A value of zero means to use the airspeed as is.
   "AHRS_TRIM_X"    , // AHRS Trim Roll, Compensates for the roll angle difference between the control board and the frame. Positive values make the vehicle roll right.
   "AHRS_TRIM_Y"    , // AHRS Trim Pitch, Compensates for the pitch angle difference between the control board and the frame. Positive values make the vehicle pitch up/back.
   "AHRS_TRIM_Z"    , // AHRS Trim Yaw, Not Used                           
   "AHRS_ORIENTATION", // Board Orientation, Overall board orientation relative to the standard orientation for the board type. This rotates the IMU and compass readings to allow the board to be oriented in your vehicle at any 90 or 45 degree angle. This option takes affect on next boot. After changing you will need to re-level your vehicle.
   "AHRS_COMP_BETA" , // AHRS Velocity Complementary Filter Beta Coefficient, This controls the time constant for the cross-over frequency used to fuse AHRS (airspeed and heading) and GPS data to estimate ground velocity. Time constant is 0.1/beta. A larger time constant will use GPS data less and a small time constant will use air data less.
   "AHRS_GPS_MINSATS", // AHRS GPS Minimum satellites, Minimum number of satellites visible to use GPS for velocity based corrections attitude correction. This defaults to 6, which is about the point at which the velocity numbers from a GPS become too unreliable for accurate correction of the accelerometers.
   "AHRS_EKF_TYPE"  , // Use NavEKF Kalman filter for attitude and position estimation, This controls whether the NavEKF Kalman filter is used for attitude and position estimation and whether fallback to the DCM algorithm is allowed. Note that on copters "disabled" is not available, and will be the same as "enabled - no fallback"
   "MNT_DEFLT_MODE" , // Mount default operating mode, Mount default operating mode on startup and after control is returned from autopilot
   "MNT_RETRACT_X"  , // Mount roll angle when in retracted position, Mount roll angle when in retracted position
   "MNT_RETRACT_Y"  , // Mount tilt/pitch angle when in retracted position, Mount tilt/pitch angle when in retracted position
   "MNT_RETRACT_Z"  , // Mount yaw/pan angle when in retracted position, Mount yaw/pan angle when in retracted position
   "MNT_NEUTRAL_X"  , // Mount roll angle when in neutral position, Mount roll angle when in neutral position
   "MNT_NEUTRAL_Y"  , // Mount tilt/pitch angle when in neutral position, Mount tilt/pitch angle when in neutral position
   "MNT_NEUTRAL_Z"  , // Mount pan/yaw angle when in neutral position, Mount pan/yaw angle when in neutral position
   "MNT_STAB_ROLL"  , // Stabilize mount's roll angle, enable roll stabilisation relative to Earth
   "MNT_STAB_TILT"  , // Stabilize mount's pitch/tilt angle, enable tilt/pitch stabilisation relative to Earth
   "MNT_STAB_PAN"   , // Stabilize mount pan/yaw angle, enable pan/yaw stabilisation relative to Earth
   "MNT_RC_IN_ROLL" , // roll RC input channel, 0 for none, any other for the RC channel to be used to control roll movements
   "MNT_ANGMIN_ROL" , // Minimum roll angle, Minimum physical roll angular position of mount.
   "MNT_ANGMAX_ROL" , // Maximum roll angle, Maximum physical roll angular position of the mount
   "MNT_RC_IN_TILT" , // tilt (pitch) RC input channel, 0 for none, any other for the RC channel to be used to control tilt (pitch) movements
   "MNT_ANGMIN_TIL" , // Minimum tilt angle, Minimum physical tilt (pitch) angular position of mount.
   "MNT_ANGMAX_TIL" , // Maximum tilt angle, Maximum physical tilt (pitch) angular position of the mount
   "MNT_RC_IN_PAN"  , // pan (yaw) RC input channel, 0 for none, any other for the RC channel to be used to control pan (yaw) movements
   "MNT_ANGMIN_PAN" , // Minimum pan angle, Minimum physical pan (yaw) angular position of mount.
   "MNT_ANGMAX_PAN" , // Maximum pan angle, Maximum physical pan (yaw) angular position of the mount
   "MNT_JSTICK_SPD" , // mount joystick speed, 0 for position control, small for low speeds, 100 for max speed. A good general value is 10 which gives a movement speed of 3 degrees per second.
   "MNT_LEAD_RLL"   , // Roll stabilization lead time, Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate, compensating for servo delay. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.
   "MNT_LEAD_PTCH"  , // Pitch stabilization lead time, Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.
   "MNT_TYPE"       , // Mount Type, Mount Type (None, Servo or MAVLink)   
   "MNT2_DEFLT_MODE", // Mount default operating mode, Mount default operating mode on startup and after control is returned from autopilot
   "MNT2_RETRACT_X" , // Mount2 roll angle when in retracted position, Mount2 roll angle when in retracted position
   "MNT2_RETRACT_Y" , // Mount2 tilt/pitch angle when in retracted position, Mount2 tilt/pitch angle when in retracted position
   "MNT2_RETRACT_Z" , // Mount2 yaw/pan angle when in retracted position, Mount2 yaw/pan angle when in retracted position
   "MNT2_NEUTRAL_X" , // Mount2 roll angle when in neutral position, Mount2 roll angle when in neutral position
   "MNT2_NEUTRAL_Y" , // Mount2 tilt/pitch angle when in neutral position, Mount2 tilt/pitch angle when in neutral position
   "MNT2_NEUTRAL_Z" , // Mount2 pan/yaw angle when in neutral position, Mount2 pan/yaw angle when in neutral position
   "MNT2_STAB_ROLL" , // Stabilize Mount2's roll angle, enable roll stabilisation relative to Earth
   "MNT2_STAB_TILT" , // Stabilize Mount2's pitch/tilt angle, enable tilt/pitch stabilisation relative to Earth
   "MNT2_STAB_PAN"  , // Stabilize mount2 pan/yaw angle, enable pan/yaw stabilisation relative to Earth
   "MNT2_RC_IN_ROLL", // Mount2's roll RC input channel, 0 for none, any other for the RC channel to be used to control roll movements
   "MNT2_ANGMIN_ROL", // Mount2's minimum roll angle, Mount2's minimum physical roll angular position
   "MNT2_ANGMAX_ROL", // Mount2's maximum roll angle, Mount2's maximum physical roll angular position
   "MNT2_RC_IN_TILT", // Mount2's tilt (pitch) RC input channel, 0 for none, any other for the RC channel to be used to control tilt (pitch) movements
   "MNT2_ANGMIN_TIL", // Mount2's minimum tilt angle, Mount2's minimum physical tilt (pitch) angular position
   "MNT2_ANGMAX_TIL", // Mount2's maximum tilt angle, Mount2's maximum physical tilt (pitch) angular position
   "MNT2_RC_IN_PAN" , // Mount2's pan (yaw) RC input channel, 0 for none, any other for the RC channel to be used to control pan (yaw) movements
   "MNT2_ANGMIN_PAN", // Mount2's minimum pan angle, Mount2's minimum physical pan (yaw) angular position
   "MNT2_ANGMAX_PAN", // Mount2's maximum pan angle, MOunt2's maximum physical pan (yaw) angular position
   "MNT2_LEAD_RLL"  , // Mount2's Roll stabilization lead time, Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate, compensating for servo delay. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.
   "MNT2_LEAD_PTCH" , // Mount2's Pitch stabilization lead time, Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.
   "MNT2_TYPE"      , // Mount2 Type, Mount Type (None, Servo or MAVLink)  
   "LOG_BACKEND_TYPE", // DataFlash Backend Storage type, 0 for None, 1 for File, 2 for dataflash mavlink, 3 for both file and dataflash
   "LOG_FILE_BUFSIZE", // Maximum DataFlash File Backend buffer size (in kilobytes), The DataFlash_File backend uses a buffer to store data before writing to the block device.  Raising this value may reduce "gaps" in your SD card logging.  This buffer size may be reduced depending on available memory.  PixHawk requires at least 4 kilobytes.  Maximum value available here is 64 kilobytes.
   "LOG_DISARMED"   , // Enable logging while disarmed, If LOG_DISARMED is set to 1 then logging will be enabled while disarmed. This can make for very large logfiles but can help a lot when tracking down startup issues
   "LOG_REPLAY"     , // Enable logging of information needed for Replay, If LOG_REPLAY is set to 1 then the EKF2 state estimator will log detailed information needed for diagnosing problems with the Kalman filter. It is suggested that you also raise LOG_FILE_BUFSIZE to give more buffer space for logging and use a high quality microSD card to ensure no sensor data is lost
   "BATT_MONITOR"   , // Battery monitoring, Controls enabling monitoring of the battery's voltage and current
   "BATT_VOLT_PIN"  , // Battery Voltage sensing pin, Setting this to 0 ~ 13 will enable battery voltage sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 13. On the PX4 it should be set to 100. On the Pixhawk powered from the PM connector it should be set to 2.
   "BATT_CURR_PIN"  , // Battery Current sensing pin, Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 12. On the PX4 it should be set to 101. On the Pixhawk powered from the PM connector it should be set to 3.
   "BATT_VOLT_MULT" , // Voltage Multiplier, Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick on APM2 or Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX4 using the PX4IO power supply this should be set to 1.
   "BATT_AMP_PERVOLT", // Amps per volt, Number of amps that a 1V reading on the current sensor corresponds to. On the APM2 or Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.
   "BATT_AMP_OFFSET", // AMP offset, Voltage offset at zero current on current sensor
   "BATT_CAPACITY"  , // Battery capacity, Capacity of the battery in mAh when full
   "BATT_WATT_MAX"  , // Maximum allowed power (Watts), If battery wattage (voltage * current) exceeds this value then the system will reduce max throttle (THR_MAX, TKOFF_THR_MAX and THR_MIN for reverse thrust) to satisfy this limit. This helps limit high current to low C rated batteries regardless of battery voltage. The max throttle will slowly grow back to THR_MAX (or TKOFF_THR_MAX ) and THR_MIN if demanding the current max and under the watt max. Use 0 to disable.
   "BATT2_MONITOR"  , // Battery monitoring, Controls enabling monitoring of the battery's voltage and current
   "BATT2_VOLT_PIN" , // Battery Voltage sensing pin, Setting this to 0 ~ 13 will enable battery voltage sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 13. On the PX4 it should be set to 100. On the Pixhawk powered from the PM connector it should be set to 2.
   "BATT2_CURR_PIN" , // Battery Current sensing pin, Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 12. On the PX4 it should be set to 101. On the Pixhawk powered from the PM connector it should be set to 3.
   "BATT2_VOLT_MULT", // Voltage Multiplier, Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick on APM2 or Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX4 using the PX4IO power supply this should be set to 1.
   "BATT2_AMP_PERVOL", // Amps per volt, Number of amps that a 1V reading on the current sensor corresponds to. On the APM2 or Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.
   "BATT2_AMP_OFFSET", // AMP offset, Voltage offset at zero current on current sensor
   "BATT2_CAPACITY" , // Battery capacity, Capacity of the battery in mAh when full
   "BATT2_WATT_MAX" , // Maximum allowed current, If battery wattage (voltage * current) exceeds this value then the system will reduce max throttle (THR_MAX, TKOFF_THR_MAX and THR_MIN for reverse thrust) to satisfy this limit. This helps limit high current to low C rated batteries regardless of battery voltage. The max throttle will slowly grow back to THR_MAX (or TKOFF_THR_MAX ) and THR_MIN if demanding the current max and under the watt max. Use 0 to disable.
   "BRD_PWM_COUNT"  , // Auxiliary pin config, Control assigning of FMU pins to PWM output, timer capture and GPIO. All unassigned pins can be used for GPIO
   "BRD_SER1_RTSCTS", // Serial 1 flow control, Enable flow control on serial 1 (telemetry 1) on Pixhawk. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup. Note that the PX4v1 does not have hardware flow control pins on this port, so you should leave this disabled.
   "BRD_SER2_RTSCTS", // Serial 2 flow control, Enable flow control on serial 2 (telemetry 2) on Pixhawk and PX4. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.
   "BRD_SAFETYENABLE", // Enable use of safety arming switch, This controls the default state of the safety switch at startup. When set to 1 the safety switch will start in the safe state (flashing) at boot. When set to zero the safety switch will start in the unsafe state (solid) at startup. Note that if a safety switch is fitted the user can still control the safety state after startup using the switch. The safety state can also be controlled in software using a MAVLink message.
   "BRD_SBUS_OUT"   , //  SBUS output rate, This sets the SBUS output frame rate in Hz
   "BRD_SERIAL_NUM" , // User-defined serial number, User-defined serial number of this vehicle, it can be any arbitrary number you want and has no effect on the autopilot
   "BRD_CAN_ENABLE" , //  Enable use of UAVCAN devices, Enabling this option on a Pixhawk enables UAVCAN devices. Note that this uses about 25k of memory
   "BRD_SAFETY_MASK", // Channels to which ignore the safety switch state, A bitmask which controls what channels can move while the safety switch has not been pressed
   "BRD_IMU_TARGTEMP", // Target IMU temperature, This sets the target IMU temperature for boards with controllable IMU heating units. A value of -1 disables heating.
   "BRD_TYPE"       , // Board type, This allows selection of a PX4 or VRBRAIN board type. If set to zero then the board type is auto-detected (PX4)
   "SPRAY_ENABLE"   , // Sprayer enable/disable, Allows you to enable (1) or disable (0) the sprayer
   "SPRAY_PUMP_RATE", // Pump speed, Desired pump speed when travelling 1m/s expressed as a percentage
   "SPRAY_SPINNER"  , // Spinner rotation speed, Spinner's rotation speed in PWM (a higher rate will disperse the spray over a wider area horizontally)
   "SPRAY_SPEED_MIN", // Speed minimum, Speed minimum at which we will begin spraying
   "SPRAY_PUMP_MIN" , // Pump speed minimum, Minimum pump speed expressed as a percentage
   "GND_ABS_PRESS"  , // Absolute Pressure, calibrated ground pressure in Pascals
   "GND_TEMP"       , // ground temperature, calibrated ground temperature in degrees Celsius
   "GND_ALT_OFFSET" , // altitude offset, altitude offset in meters added to barometric altitude. This is used to allow for automatic adjustment of the base barometric altitude by a ground station equipped with a barometer. The value is added to the barometric altitude read by the aircraft. It is automatically reset to 0 when the barometer is calibrated on each reboot or when a preflight calibration is performed.
   "GND_PRIMARY"    , // Primary barometer, This selects which barometer will be the primary if multiple barometers are found
   "GPS_TYPE"       , // GPS type, GPS type                                
   "GPS_TYPE2"      , // 2nd GPS type, GPS type of 2nd GPS                 
   "GPS_NAVFILTER"  , // Navigation filter setting, Navigation filter engine setting
   "GPS_AUTO_SWITCH", // Automatic Switchover Setting, Automatic switchover to GPS reporting best lock
   "GPS_MIN_DGPS"   , // Minimum Lock Type Accepted for DGPS, Sets the minimum type of differential GPS corrections required before allowing to switch into DGPS mode.
   "GPS_SBAS_MODE"  , // SBAS Mode, This sets the SBAS (satellite based augmentation system) mode if available on this GPS. If set to 2 then the SBAS mode is not changed in the GPS. Otherwise the GPS will be reconfigured to enable/disable SBAS. Disabling SBAS may be worthwhile in some parts of the world where an SBAS signal is available but the baseline is too long to be useful.
   "GPS_MIN_ELEV"   , // Minimum elevation, This sets the minimum elevation of satellites above the horizon for them to be used for navigation. Setting this to -100 leaves the minimum elevation set to the GPS modules default.
   "GPS_INJECT_TO"  , // Destination for GPS_INJECT_DATA MAVLink packets, The GGS can send raw serial packets to inject data to multiple GPSes.
   "GPS_SBP_LOGMASK", // Swift Binary Protocol Logging Mask, Masked with the SBP msg_type field to determine whether SBR1/SBR2 data is logged
   "GPS_RAW_DATA"   , // Raw data logging, Enable logging of RXM raw data from uBlox which includes carrier phase and pseudo range information. This allows for post processing of dataflash logs for more precise positioning. Note that this requires a raw capable uBlox such as the 6P or 6T.
   "GPS_GNSS_MODE"  , // GNSS system configuration, Bitmask for what GNSS system to use on the first GPS (all unchecked or zero to leave GPS as configured)
   "GPS_SAVE_CFG"   , // Save GPS configuration, Determines whether the configuration for this GPS should be written to non-volatile memory on the GPS. Currently working for UBlox 6 series and above.
   "GPS_GNSS_MODE2" , // GNSS system configuration, Bitmask for what GNSS system to use on the second GPS (all unchecked or zero to leave GPS as configured)
   "GPS_AUTO_CONFIG", // Automatic GPS configuration, Controls if the autopilot should automatically configure the GPS based on the parameters and default settings
   "SCHED_DEBUG"    , // Scheduler debug level, Set to non-zero to enable scheduler debug messages. When set to show "Slips" the scheduler will display a message whenever a scheduled task is delayed due to too much CPU load. When set to ShowOverruns the scheduled will display a message whenever a task takes longer than the limit promised in the task table.
   "SCHED_LOOP_RATE", // Scheduling main loop rate, This controls the rate of the main control loop in Hz. This should only be changed by developers. This only takes effect on restart
   "FENCE_ENABLE"   , // Fence enable/disable, Allows you to enable (1) or disable (0) the fence functionality
   "FENCE_TYPE"     , // Fence Type, Enabled fence types held as bitmask   
   "FENCE_ACTION"   , // Fence Action, What action should be taken when fence is breached
   "FENCE_ALT_MAX"  , // Fence Maximum Altitude, Maximum altitude allowed before geofence triggers
   "FENCE_RADIUS"   , // Circular Fence Radius, Circle fence radius which when breached will cause an RTL
   "FENCE_MARGIN"   , // Fence Margin, Distance that autopilot's should maintain from the fence to avoid a breach
   "FENCE_TOTAL"    , // Fence polygon point total, Number of polygon points saved in eeprom (do not update manually)
   "AVOID_ENABLE"   , // Avoidance control enable/disable, Enabled/disable stopping at fence
   "RALLY_TOTAL"    , // Rally Total, Number of rally points currently loaded
   "RALLY_LIMIT_KM" , // Rally Limit, Maximum distance to rally point. If the closest rally point is more than this number of kilometers from the current position and the home location is closer than any of the rally points from the current position then do RTL to home rather than to the closest rally point. This prevents a leftover rally point from a different airfield being used accidentally. If this is set to 0 then the closest rally point is always used.
   "RALLY_INCL_HOME", // Rally Include Home, Controls if Home is included as a Rally point (i.e. as a safe landing place) for RTL
   "H_SV1_POS"      , // Servo 1 Position, Angular location of swash servo #1
   "H_SV2_POS"      , // Servo 2 Position, Angular location of swash servo #2
   "H_SV3_POS"      , // Servo 3 Position, Angular location of swash servo #3
   "H_TAIL_TYPE"    , // Tail Type, Tail type selection.  Simpler yaw controller used if external gyro is selected
   "H_SWASH_TYPE"   , // Swash Type, Swash Type Setting - either 3-servo CCPM or H1 Mechanical Mixing
   "H_GYR_GAIN"     , // External Gyro Gain, PWM sent to external gyro on ch7 when tail type is Servo w/ ExtGyro
   "H_PHANG"        , // Swashplate Phase Angle Compensation, Phase angle correction for rotor head.  If pitching the swash forward induces a roll, this can be correct the problem
   "H_COLYAW"       , // Collective-Yaw Mixing, Feed-forward compensation to automatically add rudder input when collective pitch is increased. Can be positive or negative depending on mechanics.
   "H_FLYBAR_MODE"  , // Flybar Mode Selector, Flybar present or not.  Affects attitude controller used during ACRO flight mode
   "H_TAIL_SPEED"   , // Direct Drive VarPitch Tail ESC speed, Direct Drive VarPitch Tail ESC speed.  Only used when TailType is DirectDrive VarPitch
   "H_GYR_GAIN_ACRO", // External Gyro Gain for ACRO, PWM sent to external gyro on ch7 when tail type is Servo w/ ExtGyro. A value of zero means to use H_GYR_GAIN
   "H_RSC_PWM_MIN"  , // RSC PWM output miniumum, This sets the PWM output on RSC channel for maximum rotor speed
   "H_RSC_PWM_MAX"  , // RSC PWM output maxiumum, This sets the PWM output on RSC channel for miniumum rotor speed
   "H_RSC_PWM_REV"  , // RSC PWM reversal, This controls reversal of the RSC channel output
   "MOT_YAW_HEADROOM", // Matrix Yaw Min, Yaw control is given at least this pwm range
   "MOT_THST_EXPO"  , // Thrust Curve Expo, Motor thrust curve exponent (from 0 for linear to 1.0 for second order curve)
   "MOT_SPIN_MAX"   , // Motor Spin maximum, Point at which the thrust saturates expressed as a number from 0 to 1 in the entire output range
   "MOT_BAT_VOLT_MAX", // Battery voltage compensation maximum voltage, Battery voltage compensation maximum voltage (voltage above this will have no additional scaling effect on thrust).  Recommend 4.4 * cell count, 0 = Disabled
   "MOT_BAT_VOLT_MIN", // Battery voltage compensation minimum voltage, Battery voltage compensation minimum voltage (voltage below this will have no additional scaling effect on thrust).  Recommend 3.5 * cell count, 0 = Disabled
   "MOT_BAT_CURR_MAX", // Motor Current Max, Maximum current over which maximum throttle is limited (0 = Disabled)
   "MOT_PWM_TYPE"   , // Output PWM type, This selects the output PWM type, allowing for normal PWM continuous output or OneShot125
   "MOT_PWM_MIN"    , // PWM output miniumum, This sets the min PWM output value that will ever be output to the motors, 0 = use input RC3_MIN
   "MOT_PWM_MAX"    , // PWM output maximum, This sets the max PWM value that will ever be output to the motors, 0 = use input RC3_MAX
   "MOT_SPIN_MIN"   , // Motor Spin minimum, Point at which the thrust starts expressed as a number from 0 to 1 in the entire output range
   "MOT_SPIN_ARM"   , // Motor Spin armed, Point at which the motors start to spin expressed as a number from 0 to 1 in the entire output range
   "MOT_BAT_CURR_TC", // Motor Current Max Time Constant, Time constant used to limit the maximum current
   "MOT_THST_HOVER" , // Thrust Hover Value, Motor thrust needed to hover expressed as a number from 0 to 1
   "MOT_HOVER_LEARN", // Hover Value Learning, Enable/Disable automatic learning of hover throttle
   "RCMAP_ROLL"     , // Roll channel, Roll channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Roll is normally on channel 1, but you can move it to any channel with this parameter.  Reboot is required for changes to take effect.
   "RCMAP_PITCH"    , // Pitch channel, Pitch channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Pitch is normally on channel 2, but you can move it to any channel with this parameter.  Reboot is required for changes to take effect.
   "RCMAP_THROTTLE" , // Throttle channel, Throttle channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Throttle is normally on channel 3, but you can move it to any channel with this parameter. Warning APM 2.X: Changing the throttle channel could produce unexpected fail-safe results if connection between receiver and on-board PPM Encoder is lost. Disabling on-board PPM Encoder is recommended.  Reboot is required for changes to take effect.
   "RCMAP_YAW"      , // Yaw channel, Yaw channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Yaw (also known as rudder) is normally on channel 4, but you can move it to any channel with this parameter.  Reboot is required for changes to take effect.
   "EKF_ENABLE"     , // Enable EKF1, This enables EKF1 to be disabled when using alternative algorithms. When disabling it, the alternate EKF2 estimator must be enabled by setting EK2_ENABLED = 1 and flight control algorithms must be set to use the alternative estimator by setting AHRS_EKF_TYPE = 2.
   "EKF_VELNE_NOISE", // GPS horizontal velocity measurement noise scaler, This is the scaler that is applied to the speed accuracy reported by the receiver to estimate the horizontal velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then a speed accuracy of 1 is assumed. Increasing it reduces the weighting on these measurements.
   "EKF_VELD_NOISE" , // GPS vertical velocity measurement noise scaler, This is the scaler that is applied to the speed accuracy reported by the receiver to estimate the vertical velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then a speed accuracy of 1 is assumed. Increasing it reduces the weighting on this measurement.
   "EKF_POSNE_NOISE", // GPS horizontal position measurement noise (m), This is the RMS value of noise in the GPS horizontal position measurements. Increasing it reduces the weighting on these measurements.
   "EKF_ALT_NOISE"  , // Altitude measurement noise (m), This is the RMS value of noise in the altitude measurement. Increasing it reduces the weighting on this measurement.
   "EKF_MAG_NOISE"  , // Magnetometer measurement noise (Gauss), This is the RMS value of noise in magnetometer measurements. Increasing it reduces the weighting on these measurements.
   "EKF_EAS_NOISE"  , // Equivalent airspeed measurement noise (m/s), This is the RMS value of noise in equivalent airspeed measurements. Increasing it reduces the weighting on these measurements.
   "EKF_WIND_PNOISE", // Wind velocity process noise (m/s^2), This noise controls the growth of wind state error estimates. Increasing it makes wind estimation faster and noisier.
   "EKF_WIND_PSCALE", // Height rate to wind procss noise scaler, Increasing this parameter increases how rapidly the wind states adapt when changing altitude, but does make wind speed estimation noiser.
   "EKF_GYRO_PNOISE", // Rate gyro noise (rad/s), This noise controls the growth of estimated error due to gyro measurement errors excluding bias. Increasing it makes the flter trust the gyro measurements less and other measurements more.
   "EKF_ACC_PNOISE" , // Accelerometer noise (m/s^2), This noise controls the growth of estimated error due to accelerometer measurement errors excluding bias. Increasing it makes the flter trust the accelerometer measurements less and other measurements more.
   "EKF_GBIAS_PNOISE", // Rate gyro bias process noise (rad/s), This noise controls the growth of gyro bias state error estimates. Increasing it makes rate gyro bias estimation faster and noisier.
   "EKF_ABIAS_PNOISE", // Accelerometer bias process noise (m/s^2), This noise controls the growth of the vertical acelerometer bias state error estimate. Increasing it makes accelerometer bias estimation faster and noisier.
   "EKF_MAGE_PNOISE", // Earth magnetic field process noise (gauss/s), This noise controls the growth of earth magnetic field state error estimates. Increasing it makes earth magnetic field bias estimation faster and noisier.
   "EKF_MAGB_PNOISE", // Body magnetic field process noise (gauss/s), This noise controls the growth of body magnetic field state error estimates. Increasing it makes compass offset estimation faster and noisier.
   "EKF_VEL_DELAY"  , // GPS velocity measurement delay (msec), This is the number of msec that the GPS velocity measurements lag behind the inertial measurements.
   "EKF_POS_DELAY"  , // GPS position measurement delay (msec), This is the number of msec that the GPS position measurements lag behind the inertial measurements.
   "EKF_GPS_TYPE"   , // GPS mode control, This parameter controls use of GPS measurements : 0 = use 3D velocity & 2D position, 1 = use 2D velocity and 2D position, 2 = use 2D position, 3 = use no GPS (optical flow will be used if available)
   "EKF_VEL_GATE"   , // GPS velocity measurement gate size, This parameter sets the number of standard deviations applied to the GPS velocity measurement innovation consistency check. Decreasing it makes it more likely that good measurements willbe rejected. Increasing it makes it more likely that bad measurements will be accepted.
   "EKF_POS_GATE"   , // GPS position measurement gate size, This parameter sets the number of standard deviations applied to the GPS position measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
   "EKF_HGT_GATE"   , // Height measurement gate size, This parameter sets the number of standard deviations applied to the height measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
   "EKF_MAG_GATE"   , // Magnetometer measurement gate size, This parameter sets the number of standard deviations applied to the magnetometer measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
   "EKF_EAS_GATE"   , // Airspeed measurement gate size, This parameter sets the number of standard deviations applied to the airspeed measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
   "EKF_MAG_CAL"    , // Magnetometer calibration mode, EKF_MAG_CAL = 0 enables calibration based on flying speed and altitude and is the default setting for Plane users. EKF_MAG_CAL = 1 enables calibration based on manoeuvre level and is the default setting for Copter and Rover users. EKF_MAG_CAL = 2 prevents magnetometer calibration regardless of flight condition and is recommended if in-flight magnetometer calibration is unreliable.
   "EKF_GLITCH_ACCEL", // GPS glitch accel gate size (cm/s^2), This parameter controls the maximum amount of difference in horizontal acceleration between the value predicted by the filter and the value measured by the GPS before the GPS position data is rejected. If this value is set too low, then valid GPS data will be regularly discarded, and the position accuracy will degrade. If this parameter is set too high, then large GPS glitches will cause large rapid changes in position.
   "EKF_GLITCH_RAD" , // GPS glitch radius gate size (m), This parameter controls the maximum amount of difference in horizontal position (in m) between the value predicted by the filter and the value measured by the GPS before the long term glitch protection logic is activated and the filter states are reset to the new GPS position. Position steps smaller than this value will be temporarily ignored, but will then be accepted and the filter will move to the new position. Position steps larger than this value will be ignored initially, but the filter will then apply an offset to the GPS position measurement.
   "EKF_GND_GRADIENT", // Terrain Gradient % RMS, This parameter sets the RMS terrain gradient percentage assumed by the terrain height estimation. Terrain height can be estimated using optical flow and/or range finder sensor data if fitted. Smaller values cause the terrain height estimate to be slower to respond to changes in measurement. Larger values cause the terrain height estimate to be faster to respond, but also more noisy. Generally this value can be reduced if operating over very flat terrain and increased if operating over uneven terrain.
   "EKF_FLOW_NOISE" , // Optical flow measurement noise (rad/s), This is the RMS value of noise and errors in optical flow measurements. Increasing it reduces the weighting on these measurements.
   "EKF_FLOW_GATE"  , // Optical Flow measurement gate size, This parameter sets the number of standard deviations applied to the optical flow innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
   "EKF_FLOW_DELAY" , // Optical Flow measurement delay (msec), This is the number of msec that the optical flow measurements lag behind the inertial measurements. It is the time from the end of the optical flow averaging period and does not include the time delay due to the 100msec of averaging within the flow sensor.
   "EKF_RNG_GATE"   , // Range finder measurement gate size, This parameter sets the number of standard deviations applied to the range finder innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
   "EKF_MAX_FLOW"   , // Maximum valid optical flow rate, This parameter sets the magnitude maximum optical flow rate in rad/sec that will be accepted by the filter
   "EKF_FALLBACK"   , // Fallback strictness, This parameter controls the conditions necessary to trigger a fallback to DCM and INAV. A value of 1 will cause fallbacks to occur on loss of GPS and other conditions. A value of 0 will trust the EKF more.
   "EKF_ALT_SOURCE" , // Primary height source, This parameter controls which height sensor is used by the EKF during optical flow navigation (when EKF_GPS_TYPE = 3). A value of will 0 cause it to always use baro altitude. A value of 1 will cause it to use range finder if available.
   "EKF_GPS_CHECK"  , // GPS preflight check, 1 byte bitmap of GPS preflight checks to perform. Set to 0 to bypass all checks. Set to 255 perform all checks. Set to 3 to check just the number of satellites and HDoP. Set to 31 for the most rigorous checks that will still allow checks to pass when the copter is moving, eg launch from a boat.
   "EK2_ENABLE"     , // Enable EKF2, This enables EKF2. Enabling EKF2 only makes the maths run, it does not mean it will be used for flight control. To use it for flight control set AHRS_EKF_TYPE=2. A reboot or restart will need to be performed after changing the value of EK2_ENABLE for it to take effect.
   "EK2_GPS_TYPE"   , // GPS mode control, This controls use of GPS measurements : 0 = use 3D velocity & 2D position, 1 = use 2D velocity and 2D position, 2 = use 2D position, 3 = use no GPS (optical flow will be used if available)
   "EK2_VELNE_M_NSE", // GPS horizontal velocity measurement noise (m/s), This sets a lower limit on the speed accuracy reported by the GPS receiver that is used to set horizontal velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then the parameter value will be used. Increasing it reduces the weighting of the GPS horizontal velocity measurements.
   "EK2_VELD_M_NSE" , // GPS vertical velocity measurement noise (m/s), This sets a lower limit on the speed accuracy reported by the GPS receiver that is used to set vertical velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then the parameter value will be used. Increasing it reduces the weighting of the GPS vertical velocity measurements.
   "EK2_VEL_I_GATE" , // GPS velocity innovation gate size, This sets the percentage number of standard deviations applied to the GPS velocity measurement innovation consistency check. Decreasing it makes it more likely that good measurements willbe rejected. Increasing it makes it more likely that bad measurements will be accepted.
   "EK2_POSNE_M_NSE", // GPS horizontal position measurement noise (m), This sets the GPS horizontal position observation noise. Increasing it reduces the weighting of GPS horizontal position measurements.
   "EK2_POS_I_GATE" , // GPS position measurement gate size, This sets the percentage number of standard deviations applied to the GPS position measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
   "EK2_GLITCH_RAD" , // GPS glitch radius gate size (m), This controls the maximum radial uncertainty in position between the value predicted by the filter and the value measured by the GPS before the filter position and velocity states are reset to the GPS. Making this value larger allows the filter to ignore larger GPS glitches but also means that non-GPS errors such as IMU and compass can create a larger error in position before the filter is forced back to the GPS position.
   "EK2_GPS_DELAY"  , // GPS measurement delay (msec), This is the number of msec that the GPS measurements lag behind the inertial measurements.
   "EK2_ALT_SOURCE" , // Primary height source, This parameter controls the primary height sensor used by the EKF. If the selected option cannot be used, it will default to Baro as the primary height source. Setting 0 will use the baro altitude at all times. Setting 1 uses the range finder and is only available in combination with optical flow navigation (EK2_GPS_TYPE = 3). Setting 2 uses GPS. NOTE - the EK2_RNG_USE_HGT parameter can be used to switch to range-finder when close to the ground.
   "EK2_ALT_M_NSE"  , // Altitude measurement noise (m), This is the RMS value of noise in the altitude measurement. Increasing it reduces the weighting of the baro measurement and will make the filter respond more slowly to baro measurement errors, but will make it more sensitive to GPS and accelerometer errors.
   "EK2_HGT_I_GATE" , // Height measurement gate size, This sets the percentage number of standard deviations applied to the height measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
   "EK2_HGT_DELAY"  , // Height measurement delay (msec), This is the number of msec that the Height measurements lag behind the inertial measurements.
   "EK2_MAG_M_NSE"  , // Magnetometer measurement noise (Gauss), This is the RMS value of noise in magnetometer measurements. Increasing it reduces the weighting on these measurements.
   "EK2_MAG_CAL"    , // Magnetometer calibration mode, EKF_MAG_CAL = 0 enables calibration when airborne and is the default setting for Plane users. EKF_MAG_CAL = 1 enables calibration when manoeuvreing. EKF_MAG_CAL = 2 prevents magnetometer calibration regardless of flight condition, is recommended if the external magnetic field is varying and is the default for rovers. EKF_MAG_CAL = 3 enables calibration when the first in-air field and yaw reset has completed and is the default for copters. EKF_MAG_CAL = 4 enables calibration all the time. This determines when the filter will use the 3-axis magnetometer fusion model that estimates both earth and body fixed magnetic field states. This model is only suitable for use when the external magnetic field environment is stable.
   "EK2_MAG_I_GATE" , // Magnetometer measurement gate size, This sets the percentage number of standard deviations applied to the magnetometer measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
   "EK2_EAS_M_NSE"  , // Equivalent airspeed measurement noise (m/s), This is the RMS value of noise in equivalent airspeed measurements used by planes. Increasing it reduces the weighting of airspeed measurements and will make wind speed estimates less noisy and slower to converge. Increasing also increases navigation errors when dead-reckoning without GPS measurements.
   "EK2_EAS_I_GATE" , // Airspeed measurement gate size, This sets the percentage number of standard deviations applied to the airspeed measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
   "EK2_RNG_M_NSE"  , // Range finder measurement noise (m), This is the RMS value of noise in the range finder measurement. Increasing it reduces the weighting on this measurement.
   "EK2_RNG_I_GATE" , // Range finder measurement gate size, This sets the percentage number of standard deviations applied to the range finder innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
   "EK2_MAX_FLOW"   , // Maximum valid optical flow rate, This sets the magnitude maximum optical flow rate in rad/sec that will be accepted by the filter
   "EK2_FLOW_M_NSE" , // Optical flow measurement noise (rad/s), This is the RMS value of noise and errors in optical flow measurements. Increasing it reduces the weighting on these measurements.
   "EK2_FLOW_I_GATE", // Optical Flow measurement gate size, This sets the percentage number of standard deviations applied to the optical flow innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
   "EK2_FLOW_DELAY" , // Optical Flow measurement delay (msec), This is the number of msec that the optical flow measurements lag behind the inertial measurements. It is the time from the end of the optical flow averaging period and does not include the time delay due to the 100msec of averaging within the flow sensor.
   "EK2_GYRO_P_NSE" , // Rate gyro noise (rad/s), This control disturbance noise controls the growth of estimated error due to gyro measurement errors excluding bias. Increasing it makes the flter trust the gyro measurements less and other measurements more.
   "EK2_ACC_P_NSE"  , // Accelerometer noise (m/s^2), This control disturbance noise controls the growth of estimated error due to accelerometer measurement errors excluding bias. Increasing it makes the flter trust the accelerometer measurements less and other measurements more.
   "EK2_GBIAS_P_NSE", // Rate gyro bias stability (rad/s/s), This state  process noise controls growth of the gyro delta angle bias state error estimate. Increasing it makes rate gyro bias estimation faster and noisier.
   "EK2_GSCL_P_NSE" , // Rate gyro scale factor stability (1/s), This noise controls the rate of gyro scale factor learning. Increasing it makes rate gyro scale factor estimation faster and noisier.
   "EK2_ABIAS_P_NSE", // Accelerometer bias stability (m/s^3), This noise controls the growth of the vertical accelerometer delta velocity bias state error estimate. Increasing it makes accelerometer bias estimation faster and noisier.
   "EK2_WIND_P_NSE" , // Wind velocity process noise (m/s^2), This state process noise controls the growth of wind state error estimates. Increasing it makes wind estimation faster and noisier.
   "EK2_WIND_PSCALE", // Height rate to wind procss noise scaler, This controls how much the process noise on the wind states is increased when gaining or losing altitude to take into account changes in wind speed and direction with altitude. Increasing this parameter increases how rapidly the wind states adapt when changing altitude, but does make wind velocity estimation noiser.
   "EK2_GPS_CHECK"  , // GPS preflight check, This is a 1 byte bitmap controlling which GPS preflight checks are performed. Set to 0 to bypass all checks. Set to 255 perform all checks. Set to 3 to check just the number of satellites and HDoP. Set to 31 for the most rigorous checks that will still allow checks to pass when the copter is moving, eg launch from a boat.
   "EK2_IMU_MASK"   , // Bitmask of active IMUs, 1 byte bitmap of IMUs to use in EKF2. A separate instance of EKF2 will be started for each IMU selected. Set to 1 to use the first IMU only (default), set to 2 to use the second IMU only, set to 3 to use the first and second IMU. Additional IMU's can be used up to a maximum of 6 if memory and processing resources permit. There may be insufficient memory and processing resources to run multiple instances. If this occurs EKF2 will fail to start.
   "EK2_CHECK_SCALE", // GPS accuracy check scaler (%), This scales the thresholds that are used to check GPS accuracy before it is used by the EKF. A value of 100 is the default. Values greater than 100 increase and values less than 100 reduce the maximum GPS error the EKF will accept. A value of 200 will double the allowable GPS error.
   "EK2_NOAID_M_NSE", // Non-GPS operation position uncertainty (m), This sets the amount of position variation that the EKF allows for when operating without external measurements (eg GPS or optical flow). Increasing this parameter makes the EKF attitude estimate less sensitive to vehicle manoeuvres but more sensitive to IMU errors.
   "EK2_LOG_MASK"   , // EKF sensor logging IMU mask, This sets the IMU mask of sensors to do full logging for
   "EK2_YAW_M_NSE"  , // Yaw measurement noise (rad), This is the RMS value of noise in yaw measurements from the magnetometer. Increasing it reduces the weighting on these measurements.
   "EK2_YAW_I_GATE" , // Yaw measurement gate size, This sets the percentage number of standard deviations applied to the magnetometer yaw measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
   "EK2_TAU_OUTPUT" , // Output complementary filter time constant (centi-sec), Sets the time constant of the output complementary filter/predictor in centi-seconds.
   "EK2_MAGE_P_NSE" , // Earth magnetic field process noise (gauss/s), This state process noise controls the growth of earth magnetic field state error estimates. Increasing it makes earth magnetic field estimation faster and noisier.
   "EK2_MAGB_P_NSE" , // Body magnetic field process noise (gauss/s), This state process noise controls the growth of body magnetic field state error estimates. Increasing it makes magnetometer bias error estimation faster and noisier.
   "EK2_RNG_USE_HGT", // Range finder switch height percentage, The range finder will be used as the primary height source when below a specified percentage of the sensor maximum as set by the RNGFND_MAX_CM parameter. Set to -1 to prevent range finder use.
   "MIS_TOTAL"      , // Total mission commands, The number of mission mission items that has been loaded by the ground station. Do not change this manually.
   "MIS_RESTART"    , // Mission Restart when entering Auto mode, Controls mission starting point when entering Auto mode (either restart from beginning of mission or resume from last command run)
   "RSSI_TYPE"      , // RSSI Type, Radio Receiver RSSI type. If your radio receiver supports RSSI of some kind, set it here, then set its associated RSSI_XXXXX parameters, if any.
   "RSSI_ANA_PIN"   , // Receiver RSSI analog sensing pin, This selects an analog pin where the receiver RSSI voltage will be read.
   "RSSI_PIN_LOW"   , // Receiver RSSI voltage low, This is the voltage value that the radio receiver will put on the RSSI_ANA_PIN when the signal strength is the weakest. Since some radio receivers put out inverted values from what you might otherwise expect, this isn't necessarily a lower value than RSSI_PIN_HIGH. 
   "RSSI_PIN_HIGH"  , // Receiver RSSI voltage high, This is the voltage value that the radio receiver will put on the RSSI_ANA_PIN when the signal strength is the strongest. Since some radio receivers put out inverted values from what you might otherwise expect, this isn't necessarily a higher value than RSSI_PIN_LOW. 
   "RSSI_CHANNEL"   , // Receiver RSSI channel number, The channel number where RSSI will be output by the radio receiver (5 and above).
   "RSSI_CHAN_LOW"  , // Receiver RSSI PWM low value, This is the PWM value that the radio receiver will put on the RSSI_CHANNEL when the signal strength is the weakest. Since some radio receivers put out inverted values from what you might otherwise expect, this isn't necessarily a lower value than RSSI_CHAN_HIGH. 
   "RSSI_CHAN_HIGH" , // Receiver RSSI PWM high value, This is the PWM value that the radio receiver will put on the RSSI_CHANNEL when the signal strength is the strongest. Since some radio receivers put out inverted values from what you might otherwise expect, this isn't necessarily a higher value than RSSI_CHAN_LOW. 
   "RNGFND_TYPE"    , // Rangefinder type, What type of rangefinder device that is connected
   "RNGFND_PIN"     , // Rangefinder pin, Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
   "RNGFND_SCALING" , // Rangefinder scaling, Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
   "RNGFND_OFFSET"  , // rangefinder offset, Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM and I2C Lidars
   "RNGFND_FUNCTION", // Rangefinder function, Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
   "RNGFND_MIN_CM"  , // Rangefinder minimum distance, Minimum distance in centimeters that rangefinder can reliably read
   "RNGFND_MAX_CM"  , // Rangefinder maximum distance, Maximum distance in centimeters that rangefinder can reliably read
   "RNGFND_STOP_PIN", // Rangefinder stop pin, Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
   "RNGFND_SETTLE"  , // Rangefinder settle time, The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
   "RNGFND_RMETRIC" , // Ratiometric, This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
   "RNGFND_PWRRNG"  , // Powersave range, This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode (if available). A value of zero means power saving is not enabled
   "RNGFND_GNDCLEAR", // Distance (in cm) from the range finder to the ground, This parameter sets the expected range measurement(in cm) that the range finder should return when the vehicle is on the ground.
   "RNGFND_ADDR"    , // Bus address of sensor, This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
   "RNGFND2_TYPE"   , // Second Rangefinder type, What type of rangefinder device that is connected
   "RNGFND2_PIN"    , // Rangefinder pin, Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
   "RNGFND2_SCALING", // Rangefinder scaling, Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
   "RNGFND2_OFFSET" , // rangefinder offset, Offset in volts for zero distance
   "RNGFND2_FUNCTION", // Rangefinder function, Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
   "RNGFND2_MIN_CM" , // Rangefinder minimum distance, Minimum distance in centimeters that rangefinder can reliably read
   "RNGFND2_MAX_CM" , // Rangefinder maximum distance, Maximum distance in centimeters that rangefinder can reliably read
   "RNGFND2_STOP_PIN", // Rangefinder stop pin, Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
   "RNGFND2_SETTLE" , // Sonar settle time, The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
   "RNGFND2_RMETRIC", // Ratiometric, This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
   "RNGFND2_GNDCLEAR", // Distance (in cm) from the second range finder to the ground, This parameter sets the expected range measurement(in cm) that the second range finder should return when the vehicle is on the ground.
   "RNGFND2_ADDR"   , // Bus address of second rangefinder, This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
   "RNGFND3_TYPE"   , // Third Rangefinder type, What type of rangefinder device that is connected
   "RNGFND3_PIN"    , // Rangefinder pin, Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
   "RNGFND3_SCALING", // Rangefinder scaling, Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
   "RNGFND3_OFFSET" , // rangefinder offset, Offset in volts for zero distance
   "RNGFND3_FUNCTION", // Rangefinder function, Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
   "RNGFND3_MIN_CM" , // Rangefinder minimum distance, Minimum distance in centimeters that rangefinder can reliably read
   "RNGFND3_MAX_CM" , // Rangefinder maximum distance, Maximum distance in centimeters that rangefinder can reliably read
   "RNGFND3_STOP_PIN", // Rangefinder stop pin, Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
   "RNGFND3_SETTLE" , // Sonar settle time, The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
   "RNGFND3_RMETRIC", // Ratiometric, This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
   "RNGFND3_GNDCLEAR", // Distance (in cm) from the third range finder to the ground, This parameter sets the expected range measurement(in cm) that the third range finder should return when the vehicle is on the ground.
   "RNGFND3_ADDR"   , // Bus address of third rangefinder, This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
   "RNGFND4_TYPE"   , // Fourth Rangefinder type, What type of rangefinder device that is connected
   "RNGFND4_PIN"    , // Rangefinder pin, Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
   "RNGFND4_SCALING", // Rangefinder scaling, Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
   "RNGFND4_OFFSET" , // rangefinder offset, Offset in volts for zero distance
   "RNGFND4_FUNCTION", // Rangefinder function, Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
   "RNGFND4_MIN_CM" , // Rangefinder minimum distance, Minimum distance in centimeters that rangefinder can reliably read
   "RNGFND4_MAX_CM" , // Rangefinder maximum distance, Maximum distance in centimeters that rangefinder can reliably read
   "RNGFND4_STOP_PIN", // Rangefinder stop pin, Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
   "RNGFND4_SETTLE" , // Sonar settle time, The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
   "RNGFND4_RMETRIC", // Ratiometric, This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
   "RNGFND4_GNDCLEAR", // Distance (in cm) from the fourth range finder to the ground, This parameter sets the expected range measurement(in cm) that the fourth range finder should return when the vehicle is on the ground.
   "RNGFND4_ADDR"   , // Bus address of fourth rangefinder, This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
   "TERRAIN_ENABLE" , // Terrain data enable, enable terrain data. This enables the vehicle storing a database of terrain data on the SD card. The terrain data is requested from the ground station as needed, and stored for later use on the SD card. To be useful the ground station must support TERRAIN_REQUEST messages and have access to a terrain database, such as the SRTM database.
   "TERRAIN_SPACING", // Terrain grid spacing, Distance between terrain grid points in meters. This controls the horizontal resolution of the terrain data that is stored on te SD card and requested from the ground station. If your GCS is using the worldwide SRTM database then a resolution of 100 meters is appropriate. Some parts of the world may have higher resolution data available, such as 30 meter data available in the SRTM database in the USA. The grid spacing also controls how much data is kept in memory during flight. A larger grid spacing will allow for a larger amount of data in memory. A grid spacing of 100 meters results in the vehicle keeping 12 grid squares in memory with each grid square having a size of 2.7 kilometers by 3.2 kilometers. Any additional grid squares are stored on the SD once they are fetched from the GCS and will be demand loaded as needed.
   "FLOW_ENABLE"    , // Optical flow enable/disable, Setting this to Enabled(1) will enable optical flow. Setting this to Disabled(0) will disable optical flow
   "FLOW_FXSCALER"  , // X axis optical flow scale factor correction, This sets the parts per thousand scale factor correction applied to the flow sensor X axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the X axis optical flow reading by 0.1%. Negative values reduce the scale factor.
   "FLOW_FYSCALER"  , // Y axis optical flow scale factor correction, This sets the parts per thousand scale factor correction applied to the flow sensor Y axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the Y axis optical flow reading by 0.1%. Negative values reduce the scale factor.
   "FLOW_ORIENT_YAW", // Flow sensor yaw alignment, Specifies the number of centi-degrees that the flow sensor is yawed relative to the vehicle. A sensor with its X-axis pointing to the right of the vehicle X axis has a positive yaw angle.
   "PLND_ENABLED"   , // Precision Land enabled/disabled and behaviour, Precision Land enabled/disabled and behaviour
   "PLND_TYPE"      , // Precision Land Type, Precision Land Type          
   "RPM_TYPE"       , // RPM type, What type of RPM sensor is connected    
   "RPM_SCALING"    , // RPM scaling, Scaling factor between sensor reading and RPM.
   "RPM_MAX"        , // Maximum RPM, Maximum RPM to report                
   "RPM_MIN"        , // Minimum RPM, Minimum RPM to report                
   "RPM_MIN_QUAL"   , // Minimum Quality, Minimum data quality to be used  
   "RPM2_TYPE"      , // Second RPM type, What type of RPM sensor is connected
   "RPM2_SCALING"   , // RPM scaling, Scaling factor between sensor reading and RPM.
   "ADSB_ENABLE"    , // Enable ADSB, Enable ADS-B                         
   "ADSB_LIST_MAX"  , // ADSB vehicle list size, ADSB list size of nearest vehicles. Longer lists take longer to refresh with lower SRx_ADSB values.
   "ADSB_LIST_RADIUS", // ADSB vehicle list radius filter, ADSB vehicle list radius filter. Vehicles detected outside this radius will be completely ignored. They will not show up in the SRx_ADSB stream to the GCS and will not be considered in any avoidance calculations.
   "ADSB_ICAO_ID"   , // ICAO_ID vehicle identifaction number, ICAO_ID unique vehicle identifaction number of this aircraft. This is a integer limited to 24bits. If set to 0 then one will be randomly generated. If set to -1 then static information is not sent, transceiver is assumed pre-programmed.
   "ADSB_EMIT_TYPE" , // Emitter type, ADSB classification for the type of vehicle emitting the transponder signal. Default value is 14 (UAV).
   "ADSB_LEN_WIDTH" , // Aircraft length and width, Aircraft length and width dimension options in Length and Width in meters. In most cases, use a value of 1 for smallest size.
   "ADSB_OFFSET_LAT", // GPS antenna lateral offset, GPS antenna lateral offset. This describes the physical location offest from center of the GPS antenna on the aircraft.
   "ADSB_OFFSET_LON", // GPS antenna longitudinal offset, GPS antenna longitudinal offset. This is usually set to 1, Applied By Sensor
   "ADSB_RF_SELECT" , // Transceiver RF selection, Transceiver RF selection for Rx enable and/or Tx enable.
   "AVD_ENABLE"     , // Enable Avoidance using ADSB, Enable Avoidance using ADSB
   "AVD_F_RCVRY"    , // Recovery behaviour after a fail event, Determines what the aircraft will do after a fail event is resolved
   "AVD_OBS_MAX"    , // Maximum number of obstacles to track, Maximum number of obstacles to track
   "AVD_W_TIME"     , // Time Horizon Warn, Aircraft velocity vectors are multiplied by this time to determine closest approach.  If this results in an approach closer than W_DIST_XY or W_DIST_Z then W_ACTION is undertaken (assuming F_ACTION is not undertaken)
   "AVD_F_TIME"     , // Time Horizon Fail, Aircraft velocity vectors are multiplied by this time to determine closest approach.  If this results in an approach closer than F_DIST_XY or F_DIST_Z then F_ACTION is undertaken
   "AVD_W_DIST_XY"  , // Distance Warn XY, Closest allowed projected distance before W_ACTION is undertaken
   "AVD_F_DIST_XY"  , // Distance Fail XY, Closest allowed projected distance before F_ACTION is undertaken
   "AVD_W_DIST_Z"   , // Distance Warn Z, Closest allowed projected distance before BEHAVIOUR_W is undertaken
   "AVD_F_DIST_Z"   , // Distance Fail Z, Closest allowed projected distance before BEHAVIOUR_F is undertaken
   "NTF_LED_BRIGHT" , // LED Brightness, Select the RGB LED brightness level. When USB is connected brightness will never be higher than low regardless of the setting.
   "NTF_BUZZ_ENABLE", // Buzzer enable, Enable or disable the buzzer. Only for Linux and PX4 based boards.
   "NTF_LED_OVERRIDE", // Setup for MAVLink LED override, This sets up the board RGB LED for override by MAVLink. Normal notify LED control is disabled
   "BTN_ENABLE"     , // Enable button reporting, This enables the button checking module. When this is disabled the parameters for setting button inputs are not visible
   "BTN_PIN1"       , // First button Pin, Digital pin number for first button input. 
   "BTN_PIN2"       , // Second button Pin, Digital pin number for second button input. 
   "BTN_PIN3"       , // Third button Pin, Digital pin number for third button input. 
   "BTN_PIN4"       , // Fourth button Pin, Digital pin number for fourth button input. 
   "BTN_REPORT_SEND", // Report send time, The duration in seconds that a BUTTON_CHANGE report is repeatedly sent to the GCS regarding a button changing state. Note that the BUTTON_CHANGE message is MAVLink2 only.
};
const float param_min[MAVLINK_XCP_PARAMETERS] = { 
        -3.40282e+38, // Eeprom format version number, This value is incremented when changes are made to the eeprom format
                 0.0, // Software Type, This is used by the ground station to recognise the software type (eg ArduPlane vs ArduCopter)
                 1.0, // MAVLink system ID of this vehicle, Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network
               252.0, // My ground station number, Allows restricting radio overrides to only come from my ground station
                 0.0, // CLI Enable, This enables/disables the checking for three carriage returns on telemetry links on startup to enter the diagnostics command line interface
                 0.0, // Throttle filter cutoff, Throttle filter cutoff (Hz) - active whenever altitude control is inactive - 0 to disable
                 0.0, // Pilot takeoff altitude, Altitude that altitude control modes will climb to when a takeoff is triggered with the throttle stick.
                 0.0, // Takeoff trigger deadzone, Offset from mid stick at which takeoff is triggered
                 0.0, // Throttle stick behavior, Bitmask containing various throttle stick options. Add up the values for options that you want.
                 0.0, // Telemetry startup delay, The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
                 0.0, // GCS PID tuning mask, bitmask of PIDs to send MAVLink PID_TUNING messages for
                 0.0, // RTL Altitude, The minimum relative altitude the model will move to before Returning to Launch.  Set to zero to return at current altitude.
                 0.5, // RTL cone slope, Defines a cone above home which determines maximum climb
                 0.0, // RTL speed, Defines the speed in cm/s which the aircraft will attempt to maintain horizontally while flying home. If this is set to zero, WPNAV_SPEED will be used instead.
                0.01, // Rangefinder gain, Used to adjust the speed with which the target altitude is changed when objects are sensed below the copter
                 0.0, // Battery Failsafe Enable, Controls whether failsafe will be invoked when battery voltage or current runs low
        -3.40282e+38, // Failsafe battery voltage, Battery voltage to trigger failsafe. Set to 0 to disable battery voltage failsafe. If the battery voltage drops below this voltage then the copter will RTL
        -3.40282e+38, // Failsafe battery milliAmpHours, Battery capacity remaining to trigger failsafe. Set to 0 to disable battery remaining failsafe. If the battery remaining drops below this level then the copter will RTL
                 0.0, // Ground Station Failsafe Enable, Controls whether failsafe will be invoked (and what action to take) when connection with Ground station is lost for at least 5 seconds. NB. The GCS Failsafe is only active when RC_OVERRIDE is being used to control the vehicle.
               100.0, // GPS Hdop Good, GPS Hdop value at or below this value represent a good position.  Used for pre-arm checks
                 0.0, // Compass enable/disable, Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass
                 0.0, // Super Simple Mode, Bitmask to enable Super Simple mode for some flight modes. Setting this to Disabled(0) will disable Super Simple Mode
                -1.0, // RTL Final Altitude, This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission.  Set to zero to land.
                 0.0, // RTL minimum climb, The vehicle will climb this many cm during the initial climb portion of the RTL
                 0.0, // Yaw behaviour during missions, Determines how the autopilot controls the yaw during missions and RTL
                 0.0, // RTL loiter time, Time (in milliseconds) to loiter above home before beginning final descent
                30.0, // Land speed, The descent speed for the final stage of landing in cm/s
                 0.0, // Land speed high, The descent speed for the first stage of landing in cm/s. If this is zero then WPNAV_SPEED_DN is used
                50.0, // Pilot maximum vertical speed, The maximum vertical velocity the pilot may request in cm/s
                50.0, // Pilot vertical acceleration, The vertical acceleration used when pilot is controlling the altitude
                 0.0, // Throttle Failsafe Enable, The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
               925.0, // Throttle Failsafe Value, The PWM level on channel 3 below which throttle sailsafe triggers
                 0.0, // Throttle deadzone, The deadzone above and below mid throttle.  Used in AltHold, Loiter, PosHold flight modes
                 0.0, // Flight Mode 1, Flight mode when Channel 5 pwm is <= 1230
                 0.0, // Flight Mode 2, Flight mode when Channel 5 pwm is >1230, <= 1360
                 0.0, // Flight Mode 3, Flight mode when Channel 5 pwm is >1360, <= 1490
                 0.0, // Flight Mode 4, Flight mode when Channel 5 pwm is >1490, <= 1620
                 0.0, // Flight Mode 5, Flight mode when Channel 5 pwm is >1620, <= 1749
                 0.0, // Flight Mode 6, Flight mode when Channel 5 pwm is >=1750
        -3.40282e+38, // Simple mode bitmask, Bitmask which holds which flight modes use simple heading mode (eg bit 0 = 1 means Flight Mode 0 uses simple mode)
             -6146.0, // Log bitmask, 4 byte bitmap of log types to enable
                 0.0, // ESC Calibration, Controls whether ArduCopter will enter ESC calibration on the next restart.  Do not adjust this parameter manually.
                 0.0, // Channel 6 Tuning, Controls which parameters (normally PID gains) are being tuned with transmitter's channel 6 knob
                 0.0, // Tuning minimum, The minimum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
                 0.0, // Tuning maximum, The maximum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
                 0.0, // Frame Orientation (+, X or V), Controls motor mixing for multicopters.  Not used for Tri or Traditional Helicopters.
                 0.0, // Channel 7 option, Select which function if performed when CH7 is above 1800 pwm
                 0.0, // Channel 8 option, Select which function if performed when CH8 is above 1800 pwm
                 0.0, // Channel 9 option, Select which function if performed when CH9 is above 1800 pwm
                 0.0, // Channel 10 option, Select which function if performed when CH10 is above 1800 pwm
                 0.0, // Channel 11 option, Select which function if performed when CH11 is above 1800 pwm
                 0.0, // Channel 12 option, Select which function if performed when CH12 is above 1800 pwm
               -65.0, // Arming check, Allows enabling or disabling of pre-arming checks of receiver, accelerometer, barometer, compass and GPS
                 0.0, // Disarm delay, Delay before automatic disarm in seconds. A value of zero disables auto disarm.
              1000.0, // Angle Max, Maximum lean angle in all flight modes
                 0.0, // RC Feel Roll/Pitch, RC feel for roll/pitch which controls vehicle response to user input with 0 being extremely soft and 100 being crisp
                 4.0, // PosHold braking rate, PosHold flight mode's rotation rate during braking in deg/sec
              2000.0, // PosHold braking angle max, PosHold flight mode's max lean angle during braking in centi-degrees
                 0.0, // Land repositioning, Enables user input during LAND mode, the landing phase of RTL, and auto mode landings.
                 1.0, // EKF Failsafe Action, Controls the action that will be taken when an EKF failsafe is invoked
                 0.6, // EKF failsafe variance threshold, Allows setting the maximum acceptable compass and velocity variance
                 0.0, // Crash check enable, This enables automatic crash checking. When enabled the motors will disarm if a crash is detected.
                50.0, // ESC Update Speed, This is the speed in Hertz that your ESCs will receive updates
                 1.0, // Acro Roll and Pitch P gain, Converts pilot roll and pitch into a desired rate of rotation in ACRO and SPORT mode.  Higher values mean faster rate of rotation.
                 1.0, // Acro Yaw P gain, Converts pilot yaw input into a desired rate of rotation in ACRO, Stabilize and SPORT modes.  Higher values mean faster rate of rotation.
                 0.0, // Acro Balance Roll, rate at which roll angle returns to level in acro mode.  A higher value causes the vehicle to return to level faster.
                 0.0, // Acro Balance Pitch, rate at which pitch angle returns to level in acro mode.  A higher value causes the vehicle to return to level faster.
                 0.0, // Acro Trainer, Type of trainer used in acro mode
                 0.0, // Acro Expo, Acro roll/pitch Expo to allow faster rotation when stick at edges
                 0.1, // Velocity (horizontal) P gain, Velocity (horizontal) P gain.  Converts the difference between desired velocity to a target acceleration
                0.02, // Velocity (horizontal) I gain, Velocity (horizontal) I gain.  Corrects long-term difference in desired velocity to a target acceleration
                 0.0, // Velocity (horizontal) integrator maximum, Velocity (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output
                 1.0, // Velocity (vertical) P gain, Velocity (vertical) P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
                 0.5, // Throttle acceleration controller P gain, Throttle acceleration controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
                 0.0, // Throttle acceleration controller I gain, Throttle acceleration controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
                 0.0, // Throttle acceleration controller I gain maximum, Throttle acceleration controller I gain maximum.  Constrains the maximum pwm that the I term will generate
                 0.0, // Throttle acceleration controller D gain, Throttle acceleration controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
                 1.0, // Throttle acceleration filter, Filter applied to acceleration to reduce noise.  Lower values reduce noise but add delay.
                 1.0, // Position (vertical) controller P gain, Position (vertical) controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
                 0.5, // Position (horizonal) controller P gain, Loiter position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller
                 1.0, // Autotune axis bitmask, 1-byte bitmap of axes to autotune
                0.05, // Autotune aggressiveness, Autotune aggressiveness. Defines the bounce back used to detect size of the D term.
               0.001, // AutoTune minimum D, Defines the minimum D gain
                 0.0, // Start motors before throwing is detected, Used by THROW mode. Controls whether motors will run at the speed set by THR_MIN or will be stopped when armed and waiting for the throw.
                 0.0, // Terrain Following use control, This enables terrain following for RTL and LAND flight modes. To use this option TERRAIN_ENABLE must be 1 and the GCS must  support sending terrain data to the aircraft.  In RTL the RTL_ALT will be considered a height above the terrain.  In LAND mode the vehicle will slow to LAND_SPEED 10m above terrain (instead of 10m above home).  This parameter does not affect AUTO and Guided which use a per-command flag to determine if the height is above-home, absolute or above-terrain.
                 0.0, // Takeoff navigation altitude, This is the altitude in meters above the takeoff point that attitude changes for navigation can begin
                 3.0, // Throw mode's follow up mode, Vehicle will switch to this mode after the throw is successfully completed.  Default is to stay in throw mode (18)
                 0.0, // Type of Type, Used by THROW mode. Specifies whether Copter is thrown upward or dropped.
                 0.0, // Ground Effect Compensation Enable/Disable, Ground Effect Compensation Enable/Disable
                 1.0, // Serial0 baud rate, The baud rate used on the USB console. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
                 1.0, // Console protocol selection, Control what protocol to use on the console. 
                -1.0, // Telem1 protocol selection, Control what protocol to use on the Telem1 port. Note that the Frsky options require external converter hardware. See the wiki for details.
                 1.0, // Telem1 Baud Rate, The baud rate used on the Telem1 port. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
                -1.0, // Telemetry 2 protocol selection, Control what protocol to use on the Telem2 port. Note that the Frsky options require external converter hardware. See the wiki for details.
                 1.0, // Telemetry 2 Baud Rate, The baud rate of the Telem2 port. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
                -1.0, // Serial 3 (GPS) protocol selection, Control what protocol Serial 3 (GPS) should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.
                 1.0, // Serial 3 (GPS) Baud Rate, The baud rate used for the Serial 3 (GPS). The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
                -1.0, // Serial4 protocol selection, Control what protocol Serial4 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.
                 1.0, // Serial 4 Baud Rate, The baud rate used for Serial4. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
                -1.0, // Serial5 protocol selection, Control what protocol Serial5 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.
                 1.0, // Serial 5 Baud Rate, The baud rate used for Serial5. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
               800.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                -1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
                 0.0, // RC dead-zone, dead zone around trim or bottom
               800.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                -1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
                 0.0, // RC dead-zone, dead zone around trim or bottom
               800.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                -1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
                 0.0, // RC dead-zone, dead zone around trim or bottom
               800.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                -1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
                 0.0, // RC dead-zone, dead zone around trim or bottom
               800.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                -1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
                 0.0, // RC dead-zone, dead zone around trim or bottom
                 0.0, // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
               800.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                -1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
                 0.0, // RC dead-zone, dead zone around trim or bottom
                 0.0, // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
               800.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                -1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
                 0.0, // RC dead-zone, dead zone around trim or bottom
                 0.0, // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
               800.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                -1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
                 0.0, // RC dead-zone, dead zone around trim or bottom
                 0.0, // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
               800.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                -1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
                 0.0, // RC dead-zone, dead zone around trim or bottom
                 0.0, // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
               800.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                -1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
                 0.0, // RC dead-zone, dead zone around trim or bottom
                 0.0, // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
               800.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                -1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
                 0.0, // RC dead-zone, dead zone around trim or bottom
                 0.0, // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
               800.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                -1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
                 0.0, // RC dead-zone, dead zone around trim or bottom
                 0.0, // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
               800.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                -1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
                 0.0, // RC dead-zone, dead zone around trim or bottom
                 0.0, // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
               800.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
               800.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                -1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
                 0.0, // RC dead-zone, dead zone around trim or bottom
                 0.0, // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
                 0.0, // Camera shutter (trigger) type, how to trigger the camera to take a picture
                 0.0, // Duration that shutter is held open, How long the shutter will be held open in 10ths of a second (i.e. enter 10 for 1second, 50 for 5seconds)
              1000.0, // Servo ON PWM value, PWM value to move servo to when shutter is activated
              1000.0, // Servo OFF PWM value, PWM value to move servo to when shutter is deactivated
                 0.0, // Camera trigger distance, Distance in meters between camera triggers. If this value is non-zero then the camera will trigger whenever the GPS position changes by this number of meters regardless of what mode the APM is in. Note that this parameter can also be set in an auto mission using the DO_SET_CAM_TRIGG_DIST command, allowing you to enable/disable the triggering of the camera during the flight.
                 0.0, // Relay ON value, This sets whether the relay goes high or low when it triggers. Note that you should also set RELAY_DEFAULT appropriately for your camera
                 0.0, // Minimum time between photos, Postpone shooting if previous picture was taken less than preset time(ms) ago.
                 0.0, // Maximum photo roll angle., Postpone shooting if roll is greater than limit. (0=Disable, will shoot regardless of roll).
                -1.0, // Camera feedback pin, pin number to use for save accurate camera feedback messages. If set to -1 then don't use a pin flag for this, otherwise this is a pin number which if held high after a picture trigger order, will save camera messages when camera really takes a picture. A universal camera hot shoe is needed. The pin should be held high for at least 2 milliseconds for reliable trigger detection. See also the CAM_FEEDBACK_POL option. If using AUX4 pin on a Pixhawk then a fast capture method is used that allows for the trigger time to be as short as one microsecond.
                 0.0, // Camera feedback pin polarity, Polarity for feedback pin. If this is 1 then the feedback pin should go high on trigger. If set to 0 then it should go low
                -1.0, // First Relay Pin, Digital pin number for first relay control. This is the pin used for camera control.
                -1.0, // Second Relay Pin, Digital pin number for 2nd relay control.
                -1.0, // Third Relay Pin, Digital pin number for 3rd relay control.
                -1.0, // Fourth Relay Pin, Digital pin number for 4th relay control.
                 0.0, // Default relay state, The state of the relay on boot. 
                 0.0, // EPM Enable/Disable, EPM enable/disable
              1000.0, // EPM Grab PWM, PWM value sent to EPM to initiate grabbing the cargo
              1000.0, // EPM Release PWM, PWM value sent to EPM to release the cargo
              1000.0, // EPM Neutral PWM, PWM value sent to EPM when not grabbing or releasing
                 0.0, // EPM UAVCAN Hardpoint ID, Refer to https://docs.zubax.com/opengrab_epm_v3#UAVCAN_interface
                 0.0, // Parachute release enabled or disabled, Parachute release enabled or disabled
                 0.0, // Parachute release mechanism type (relay or servo), Parachute release mechanism type (relay or servo)
              1000.0, // Parachute Servo ON PWM value, Parachute Servo PWM value when parachute is released
              1000.0, // Servo OFF PWM value, Parachute Servo PWM value when parachute is not released
                 0.0, // Parachute min altitude in meters above home, Parachute min altitude above home.  Parachute will not be released below this altitude.  0 to disable alt check.
                 0.0, // Parachute release delay, Delay in millseconds between motor stop and chute release
              1000.0, // Landing Gear Servo Retracted PWM Value, Servo PWM value when landing gear is retracted
              1000.0, // Landing Gear Servo Deployed PWM Value, Servo PWM value when landing gear is deployed
                 0.0, // Stabilize Mode Collective Point 1, Helicopter's minimum collective pitch setting at zero throttle input in Stabilize mode
                 0.0, // Stabilize Mode Collective Point 2, Helicopter's collective pitch setting at mid-low throttle input in Stabilize mode
               500.0, // Stabilize Mode Collective Point 3, Helicopter's collective pitch setting at mid-high throttle input in Stabilize mode
               500.0, // Stabilize Mode Collective Point 4, Helicopter's maximum collective pitch setting at full throttle input in Stabilize mode
                 0.0, // Acro Mode Collective Expo, Used to soften collective pitch inputs near center point in Acro mode.
              -400.0, // Compass offsets in milligauss on the X axis, Offset to be added to the compass x-axis values to compensate for metal in the frame
              -400.0, // Compass offsets in milligauss on the Y axis, Offset to be added to the compass y-axis values to compensate for metal in the frame
              -400.0, // Compass offsets in milligauss on the Z axis, Offset to be added to the compass z-axis values to compensate for metal in the frame
              -3.142, // Compass declination, An angle to compensate between the true north and magnetic north
                 0.0, // Learn compass offsets automatically, Enable or disable the automatic learning of compass offsets. You can enable learning either using a compass-only method that is suitable only for fixed wing aircraft or using the offsets learnt by the active EKF state estimator. If this option is enabled then the learnt offsets are saved when you disarm the vehicle.
                 0.0, // Use compass for yaw, Enable or disable the use of the compass (instead of the GPS) for determining heading
                 0.0, // Auto Declination, Enable or disable the automatic calculation of the declination based on gps location
                 0.0, // Motor interference compensation type, Set motor interference compensation type to disabled, throttle or current.  Do not change manually.
             -1000.0, // Motor interference compensation for body frame X axis, Multiplied by the current throttle and added to the compass's x-axis values to compensate for motor interference
             -1000.0, // Motor interference compensation for body frame Y axis, Multiplied by the current throttle and added to the compass's y-axis values to compensate for motor interference
             -1000.0, // Motor interference compensation for body frame Z axis, Multiplied by the current throttle and added to the compass's z-axis values to compensate for motor interference
                 0.0, // Compass orientation, The orientation of the compass relative to the autopilot board. This will default to the right value for each board type, but can be changed if you have an external compass. See the documentation for your external compass for the right value. The correct orientation should give the X axis forward, the Y axis to the right and the Z axis down. So if your aircraft is pointing west it should show a positive value for the Y axis, and a value close to zero for the X axis. On a PX4 or Pixhawk with an external compass the correct value is zero if the compass is correctly oriented. NOTE: This orientation is combined with any AHRS_ORIENTATION setting.
                 0.0, // Compass is attached via an external cable, Configure compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. Set to 1 if the compass is externally connected. When externally connected the COMPASS_ORIENT option operates independently of the AHRS_ORIENTATION board orientation option. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
              -400.0, // Compass2 offsets in milligauss on the X axis, Offset to be added to compass2's x-axis values to compensate for metal in the frame
              -400.0, // Compass2 offsets in milligauss on the Y axis, Offset to be added to compass2's y-axis values to compensate for metal in the frame
              -400.0, // Compass2 offsets in milligauss on the Z axis, Offset to be added to compass2's z-axis values to compensate for metal in the frame
             -1000.0, // Motor interference compensation to compass2 for body frame X axis, Multiplied by the current throttle and added to compass2's x-axis values to compensate for motor interference
             -1000.0, // Motor interference compensation to compass2 for body frame Y axis, Multiplied by the current throttle and added to compass2's y-axis values to compensate for motor interference
             -1000.0, // Motor interference compensation to compass2 for body frame Z axis, Multiplied by the current throttle and added to compass2's z-axis values to compensate for motor interference
                 0.0, // Choose primary compass, If more than one compass is available this selects which compass is the primary. Normally 0=External, 1=Internal. If no External compass is attached this parameter is ignored
              -400.0, // Compass3 offsets in milligauss on the X axis, Offset to be added to compass3's x-axis values to compensate for metal in the frame
              -400.0, // Compass3 offsets in milligauss on the Y axis, Offset to be added to compass3's y-axis values to compensate for metal in the frame
              -400.0, // Compass3 offsets in milligauss on the Z axis, Offset to be added to compass3's z-axis values to compensate for metal in the frame
             -1000.0, // Motor interference compensation to compass3 for body frame X axis, Multiplied by the current throttle and added to compass3's x-axis values to compensate for motor interference
             -1000.0, // Motor interference compensation to compass3 for body frame Y axis, Multiplied by the current throttle and added to compass3's y-axis values to compensate for motor interference
             -1000.0, // Motor interference compensation to compass3 for body frame Z axis, Multiplied by the current throttle and added to compass3's z-axis values to compensate for motor interference
        -3.40282e+38, // Compass device id, Compass device id.  Automatically detected, do not set manually
        -3.40282e+38, // Compass2 device id, Second compass's device id.  Automatically detected, do not set manually
        -3.40282e+38, // Compass3 device id, Third compass's device id.  Automatically detected, do not set manually
                 0.0, // Compass2 used for yaw, Enable or disable the second compass for determining heading.
                 0.0, // Compass2 orientation, The orientation of the second compass relative to the frame (if external) or autopilot board (if internal).
                 0.0, // Compass2 is attached via an external cable, Configure second compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
                 0.0, // Compass3 used for yaw, Enable or disable the third compass for determining heading.
                 0.0, // Compass3 orientation, The orientation of the third compass relative to the frame (if external) or autopilot board (if internal).
                 0.0, // Compass3 is attached via an external cable, Configure third compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
        -3.40282e+38, // Compass soft-iron diagonal X component, DIA_X in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
        -3.40282e+38, // Compass soft-iron diagonal Y component, DIA_Y in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
        -3.40282e+38, // Compass soft-iron diagonal Z component, DIA_Z in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
        -3.40282e+38, // Compass soft-iron off-diagonal X component, ODI_X in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
        -3.40282e+38, // Compass soft-iron off-diagonal Y component, ODI_Y in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
        -3.40282e+38, // Compass soft-iron off-diagonal Z component, ODI_Z in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
        -3.40282e+38, // Compass2 soft-iron diagonal X component, DIA_X in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
        -3.40282e+38, // Compass2 soft-iron diagonal Y component, DIA_Y in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
        -3.40282e+38, // Compass2 soft-iron diagonal Z component, DIA_Z in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
        -3.40282e+38, // Compass2 soft-iron off-diagonal X component, ODI_X in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
        -3.40282e+38, // Compass2 soft-iron off-diagonal Y component, ODI_Y in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
        -3.40282e+38, // Compass2 soft-iron off-diagonal Z component, ODI_Z in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
        -3.40282e+38, // Compass3 soft-iron diagonal X component, DIA_X in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
        -3.40282e+38, // Compass3 soft-iron diagonal Y component, DIA_Y in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
        -3.40282e+38, // Compass3 soft-iron diagonal Z component, DIA_Z in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
        -3.40282e+38, // Compass3 soft-iron off-diagonal X component, ODI_X in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
        -3.40282e+38, // Compass3 soft-iron off-diagonal Y component, ODI_Y in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
        -3.40282e+38, // Compass3 soft-iron off-diagonal Z component, ODI_Z in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
                 4.0, // Compass calibration fitness, This controls the fitness level required for a successful compass calibration. A lower value makes for a stricter fit (less likely to pass). This is the value used for the primary magnetometer. Other magnetometers get double the value.
                 0.0, // IMU Product ID, Which type of IMU is installed (read-only).
        -3.40282e+38, // Gyro offsets of X axis, Gyro sensor offsets of X axis. This is setup on each boot during gyro calibrations
        -3.40282e+38, // Gyro offsets of Y axis, Gyro sensor offsets of Y axis. This is setup on each boot during gyro calibrations
        -3.40282e+38, // Gyro offsets of Z axis, Gyro sensor offsets of Z axis. This is setup on each boot during gyro calibrations
        -3.40282e+38, // Gyro2 offsets of X axis, Gyro2 sensor offsets of X axis. This is setup on each boot during gyro calibrations
        -3.40282e+38, // Gyro2 offsets of Y axis, Gyro2 sensor offsets of Y axis. This is setup on each boot during gyro calibrations
        -3.40282e+38, // Gyro2 offsets of Z axis, Gyro2 sensor offsets of Z axis. This is setup on each boot during gyro calibrations
        -3.40282e+38, // Gyro3 offsets of X axis, Gyro3 sensor offsets of X axis. This is setup on each boot during gyro calibrations
        -3.40282e+38, // Gyro3 offsets of Y axis, Gyro3 sensor offsets of Y axis. This is setup on each boot during gyro calibrations
        -3.40282e+38, // Gyro3 offsets of Z axis, Gyro3 sensor offsets of Z axis. This is setup on each boot during gyro calibrations
                 0.8, // Accelerometer scaling of X axis, Accelerometer scaling of X axis.  Calculated during acceleration calibration routine
                 0.8, // Accelerometer scaling of Y axis, Accelerometer scaling of Y axis  Calculated during acceleration calibration routine
                 0.8, // Accelerometer scaling of Z axis, Accelerometer scaling of Z axis  Calculated during acceleration calibration routine
                -3.5, // Accelerometer offsets of X axis, Accelerometer offsets of X axis. This is setup using the acceleration calibration or level operations
                -3.5, // Accelerometer offsets of Y axis, Accelerometer offsets of Y axis. This is setup using the acceleration calibration or level operations
                -3.5, // Accelerometer offsets of Z axis, Accelerometer offsets of Z axis. This is setup using the acceleration calibration or level operations
                 0.8, // Accelerometer2 scaling of X axis, Accelerometer2 scaling of X axis.  Calculated during acceleration calibration routine
                 0.8, // Accelerometer2 scaling of Y axis, Accelerometer2 scaling of Y axis  Calculated during acceleration calibration routine
                 0.8, // Accelerometer2 scaling of Z axis, Accelerometer2 scaling of Z axis  Calculated during acceleration calibration routine
                -3.5, // Accelerometer2 offsets of X axis, Accelerometer2 offsets of X axis. This is setup using the acceleration calibration or level operations
                -3.5, // Accelerometer2 offsets of Y axis, Accelerometer2 offsets of Y axis. This is setup using the acceleration calibration or level operations
                -3.5, // Accelerometer2 offsets of Z axis, Accelerometer2 offsets of Z axis. This is setup using the acceleration calibration or level operations
                 0.8, // Accelerometer3 scaling of X axis, Accelerometer3 scaling of X axis.  Calculated during acceleration calibration routine
                 0.8, // Accelerometer3 scaling of Y axis, Accelerometer3 scaling of Y axis  Calculated during acceleration calibration routine
                 0.8, // Accelerometer3 scaling of Z axis, Accelerometer3 scaling of Z axis  Calculated during acceleration calibration routine
                -3.5, // Accelerometer3 offsets of X axis, Accelerometer3 offsets of X axis. This is setup using the acceleration calibration or level operations
                -3.5, // Accelerometer3 offsets of Y axis, Accelerometer3 offsets of Y axis. This is setup using the acceleration calibration or level operations
                -3.5, // Accelerometer3 offsets of Z axis, Accelerometer3 offsets of Z axis. This is setup using the acceleration calibration or level operations
                 0.0, // Gyro filter cutoff frequency, Filter cutoff frequency for gyroscopes. This can be set to a lower value to try to cope with very high vibration levels in aircraft. This option takes effect on the next reboot. A value of zero means no filtering (not recommended!)
                 0.0, // Accel filter cutoff frequency, Filter cutoff frequency for accelerometers. This can be set to a lower value to try to cope with very high vibration levels in aircraft. This option takes effect on the next reboot. A value of zero means no filtering (not recommended!)
                 0.0, // Use first IMU for attitude, velocity and position estimates, Use first IMU for attitude, velocity and position estimates
                 0.0, // Use second IMU for attitude, velocity and position estimates, Use second IMU for attitude, velocity and position estimates
                 0.0, // Use third IMU for attitude, velocity and position estimates, Use third IMU for attitude, velocity and position estimates
                0.05, // Stillness threshold for detecting if we are moving, Threshold to tolerate vibration to determine if vehicle is motionless. This depends on the frame type and if there is a constant vibration due to motors before launch or after landing. Total motionless is about 0.05. Suggested values: Planes/rover use 0.1, multirotors use 1, tradHeli uses 5
                 0.0, // Gyro Calibration scheme, Conrols when automatic gyro calibration is performed
                 0.0, // Accel cal trim option, Specifies how the accel cal routine determines the trims
                 1.0, // Body-fixed accelerometer, The body-fixed accelerometer to be used for trim calculation
                 0.0, // Waypoint Horizontal Speed Target, Defines the speed in cm/s which the aircraft will attempt to maintain horizontally during a WP mission
               100.0, // Waypoint Radius, Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
                 0.0, // Waypoint Climb Speed Target, Defines the speed in cm/s which the aircraft will attempt to maintain while climbing during a WP mission
                 0.0, // Waypoint Descent Speed Target, Defines the speed in cm/s which the aircraft will attempt to maintain while descending during a WP mission
                 0.0, // Loiter Horizontal Maximum Speed, Defines the maximum speed in cm/s which the aircraft will travel horizontally while in loiter mode
                50.0, // Waypoint Acceleration , Defines the horizontal acceleration in cm/s/s used during missions
                50.0, // Waypoint Vertical Acceleration, Defines the vertical acceleration in cm/s/s used during missions
               500.0, // Loiter maximum jerk, Loiter maximum jerk in cm/s/s/s
               100.0, // Loiter maximum acceleration, Loiter maximum acceleration in cm/s/s.  Higher values cause the copter to accelerate and stop more quickly.
                25.0, // Loiter minimum acceleration, Loiter minimum acceleration in cm/s/s. Higher values stop the copter more quickly when the stick is centered, but cause a larger jerk when the copter stops.
                 0.0, // Circle Radius, Defines the radius of the circle the vehicle will fly when in Circle flight mode
               -90.0, // Circle rate, Circle mode's turn rate in deg/sec.  Positive to turn clockwise, negative for counter clockwise
               500.0, // Yaw target slew rate, Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes
                 0.0, // Acceleration Max for Yaw, Maximum acceleration in yaw axis
                 0.0, // Rate Feedforward Enable, Controls whether body-frame rate feedfoward is enabled or disabled
                 0.0, // Acceleration Max for Roll, Maximum acceleration in roll axis
                 0.0, // Acceleration Max for Pitch, Maximum acceleration in pitch axis
                 0.0, // Angle Boost, Angle Boost increases output throttle as the vehicle leans to reduce loss of altitude
                 3.0, // Roll axis angle controller P gain, Roll axis angle controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate
                 3.0, // Pitch axis angle controller P gain, Pitch axis angle controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
                 3.0, // Yaw axis angle controller P gain, Yaw axis angle controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate
                 0.5, // Angle Limit (to maintain altitude) Time Constant, Angle Limit (to maintain altitude) Time Constant
                0.08, // Roll axis rate controller P gain, Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
                0.01, // Roll axis rate controller I gain, Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
                 0.0, // Roll axis rate controller I gain maximum, Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
                 0.0, // Roll axis rate controller D gain, Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
                 1.0, // Roll axis rate conroller input frequency in Hz, Roll axis rate conroller input frequency in Hz
                0.08, // Pitch axis rate controller P gain, Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output
                0.01, // Pitch axis rate controller I gain, Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
                 0.0, // Pitch axis rate controller I gain maximum, Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
                 0.0, // Pitch axis rate controller D gain, Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
                 1.0, // Pitch axis rate conroller input frequency in Hz, Pitch axis rate conroller input frequency in Hz
                 0.1, // Yaw axis rate controller P gain, Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
                0.01, // Yaw axis rate controller I gain, Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
                 0.0, // Yaw axis rate controller I gain maximum, Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
                 0.0, // Yaw axis rate controller D gain, Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
                 1.0, // Yaw axis rate conroller input frequency in Hz, Yaw axis rate conroller input frequency in Hz
                 0.1, // Throttle Mix Minimum, Throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
                 0.5, // Throttle Mix Maximum, Throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
                 0.5, // XY Acceleration filter cutoff frequency, Lower values will slow the response of the navigation controller and reduce twitchiness
                 0.0, // Raw sensor stream rate, Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
                 0.0, // Extended status stream rate to ground station, Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
                 0.0, // RC Channel stream rate to ground station, Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS_RAW to ground station
                 0.0, // Raw Control stream rate to ground station, Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station
                 0.0, // Position stream rate to ground station, Stream rate of GLOBAL_POSITION_INT to ground station
                 0.0, // Extra data type 1 stream rate to ground station, Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station
                 0.0, // Extra data type 2 stream rate to ground station, Stream rate of VFR_HUD to ground station
                 0.0, // Extra data type 3 stream rate to ground station, Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
                 0.0, // Parameter stream rate to ground station, Stream rate of PARAM_VALUE to ground station
                 0.0, // ADSB stream rate to ground station, ADSB stream rate to ground station
                 0.0, // Raw sensor stream rate, Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
                 0.0, // Extended status stream rate to ground station, Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
                 0.0, // RC Channel stream rate to ground station, Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS_RAW to ground station
                 0.0, // Raw Control stream rate to ground station, Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station
                 0.0, // Position stream rate to ground station, Stream rate of GLOBAL_POSITION_INT to ground station
                 0.0, // Extra data type 1 stream rate to ground station, Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station
                 0.0, // Extra data type 2 stream rate to ground station, Stream rate of VFR_HUD to ground station
                 0.0, // Extra data type 3 stream rate to ground station, Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
                 0.0, // Parameter stream rate to ground station, Stream rate of PARAM_VALUE to ground station
                 0.0, // ADSB stream rate to ground station, ADSB stream rate to ground station
                 0.0, // Raw sensor stream rate, Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
                 0.0, // Extended status stream rate to ground station, Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
                 0.0, // RC Channel stream rate to ground station, Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS_RAW to ground station
                 0.0, // Raw Control stream rate to ground station, Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station
                 0.0, // Position stream rate to ground station, Stream rate of GLOBAL_POSITION_INT to ground station
                 0.0, // Extra data type 1 stream rate to ground station, Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station
                 0.0, // Extra data type 2 stream rate to ground station, Stream rate of VFR_HUD to ground station
                 0.0, // Extra data type 3 stream rate to ground station, Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
                 0.0, // Parameter stream rate to ground station, Stream rate of PARAM_VALUE to ground station
                 0.0, // ADSB stream rate to ground station, ADSB stream rate to ground station
                 0.0, // Raw sensor stream rate, Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
                 0.0, // Extended status stream rate to ground station, Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
                 0.0, // RC Channel stream rate to ground station, Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS_RAW to ground station
                 0.0, // Raw Control stream rate to ground station, Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station
                 0.0, // Position stream rate to ground station, Stream rate of GLOBAL_POSITION_INT to ground station
                 0.0, // Extra data type 1 stream rate to ground station, Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station
                 0.0, // Extra data type 2 stream rate to ground station, Stream rate of VFR_HUD to ground station
                 0.0, // Extra data type 3 stream rate to ground station, Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
                 0.0, // Parameter stream rate to ground station, Stream rate of PARAM_VALUE to ground station
                 0.0, // ADSB stream rate to ground station, ADSB stream rate to ground station
                 0.0, // AHRS GPS gain, This controls how how much to use the GPS to correct the attitude. This should never be set to zero for a plane as it would result in the plane losing control in turns. For a plane please use the default value of 1.0.
                 0.0, // AHRS use GPS for navigation, This controls whether to use dead-reckoning or GPS based navigation. If set to 0 then the GPS won't be used for navigation, and only dead reckoning will be used. A value of zero should never be used for normal flight.
                 0.1, // Yaw P, This controls the weight the compass or GPS has on the heading. A higher value means the heading will track the yaw source (GPS or compass) more rapidly.
                 0.1, // AHRS RP_P, This controls how fast the accelerometers correct the attitude
                 0.0, // Maximum wind, This sets the maximum allowable difference between ground speed and airspeed. This allows the plane to cope with a failing airspeed sensor. A value of zero means to use the airspeed as is.
             -0.1745, // AHRS Trim Roll, Compensates for the roll angle difference between the control board and the frame. Positive values make the vehicle roll right.
             -0.1745, // AHRS Trim Pitch, Compensates for the pitch angle difference between the control board and the frame. Positive values make the vehicle pitch up/back.
             -0.1745, // AHRS Trim Yaw, Not Used
                 0.0, // Board Orientation, Overall board orientation relative to the standard orientation for the board type. This rotates the IMU and compass readings to allow the board to be oriented in your vehicle at any 90 or 45 degree angle. This option takes affect on next boot. After changing you will need to re-level your vehicle.
               0.001, // AHRS Velocity Complementary Filter Beta Coefficient, This controls the time constant for the cross-over frequency used to fuse AHRS (airspeed and heading) and GPS data to estimate ground velocity. Time constant is 0.1/beta. A larger time constant will use GPS data less and a small time constant will use air data less.
                 0.0, // AHRS GPS Minimum satellites, Minimum number of satellites visible to use GPS for velocity based corrections attitude correction. This defaults to 6, which is about the point at which the velocity numbers from a GPS become too unreliable for accurate correction of the accelerometers.
                 0.0, // Use NavEKF Kalman filter for attitude and position estimation, This controls whether the NavEKF Kalman filter is used for attitude and position estimation and whether fallback to the DCM algorithm is allowed. Note that on copters "disabled" is not available, and will be the same as "enabled - no fallback"
                 0.0, // Mount default operating mode, Mount default operating mode on startup and after control is returned from autopilot
              -180.0, // Mount roll angle when in retracted position, Mount roll angle when in retracted position
              -180.0, // Mount tilt/pitch angle when in retracted position, Mount tilt/pitch angle when in retracted position
              -180.0, // Mount yaw/pan angle when in retracted position, Mount yaw/pan angle when in retracted position
              -180.0, // Mount roll angle when in neutral position, Mount roll angle when in neutral position
              -180.0, // Mount tilt/pitch angle when in neutral position, Mount tilt/pitch angle when in neutral position
              -180.0, // Mount pan/yaw angle when in neutral position, Mount pan/yaw angle when in neutral position
                 0.0, // Stabilize mount's roll angle, enable roll stabilisation relative to Earth
                 0.0, // Stabilize mount's pitch/tilt angle, enable tilt/pitch stabilisation relative to Earth
                 0.0, // Stabilize mount pan/yaw angle, enable pan/yaw stabilisation relative to Earth
                 0.0, // roll RC input channel, 0 for none, any other for the RC channel to be used to control roll movements
            -18000.0, // Minimum roll angle, Minimum physical roll angular position of mount.
            -18000.0, // Maximum roll angle, Maximum physical roll angular position of the mount
                 0.0, // tilt (pitch) RC input channel, 0 for none, any other for the RC channel to be used to control tilt (pitch) movements
            -18000.0, // Minimum tilt angle, Minimum physical tilt (pitch) angular position of mount.
            -18000.0, // Maximum tilt angle, Maximum physical tilt (pitch) angular position of the mount
                 0.0, // pan (yaw) RC input channel, 0 for none, any other for the RC channel to be used to control pan (yaw) movements
            -18000.0, // Minimum pan angle, Minimum physical pan (yaw) angular position of mount.
            -18000.0, // Maximum pan angle, Maximum physical pan (yaw) angular position of the mount
                 0.0, // mount joystick speed, 0 for position control, small for low speeds, 100 for max speed. A good general value is 10 which gives a movement speed of 3 degrees per second.
                 0.0, // Roll stabilization lead time, Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate, compensating for servo delay. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.
                 0.0, // Pitch stabilization lead time, Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.
                 0.0, // Mount Type, Mount Type (None, Servo or MAVLink)
                 0.0, // Mount default operating mode, Mount default operating mode on startup and after control is returned from autopilot
              -180.0, // Mount2 roll angle when in retracted position, Mount2 roll angle when in retracted position
              -180.0, // Mount2 tilt/pitch angle when in retracted position, Mount2 tilt/pitch angle when in retracted position
              -180.0, // Mount2 yaw/pan angle when in retracted position, Mount2 yaw/pan angle when in retracted position
              -180.0, // Mount2 roll angle when in neutral position, Mount2 roll angle when in neutral position
              -180.0, // Mount2 tilt/pitch angle when in neutral position, Mount2 tilt/pitch angle when in neutral position
              -180.0, // Mount2 pan/yaw angle when in neutral position, Mount2 pan/yaw angle when in neutral position
                 0.0, // Stabilize Mount2's roll angle, enable roll stabilisation relative to Earth
                 0.0, // Stabilize Mount2's pitch/tilt angle, enable tilt/pitch stabilisation relative to Earth
                 0.0, // Stabilize mount2 pan/yaw angle, enable pan/yaw stabilisation relative to Earth
                 0.0, // Mount2's roll RC input channel, 0 for none, any other for the RC channel to be used to control roll movements
            -18000.0, // Mount2's minimum roll angle, Mount2's minimum physical roll angular position
            -18000.0, // Mount2's maximum roll angle, Mount2's maximum physical roll angular position
                 0.0, // Mount2's tilt (pitch) RC input channel, 0 for none, any other for the RC channel to be used to control tilt (pitch) movements
            -18000.0, // Mount2's minimum tilt angle, Mount2's minimum physical tilt (pitch) angular position
            -18000.0, // Mount2's maximum tilt angle, Mount2's maximum physical tilt (pitch) angular position
                 0.0, // Mount2's pan (yaw) RC input channel, 0 for none, any other for the RC channel to be used to control pan (yaw) movements
            -18000.0, // Mount2's minimum pan angle, Mount2's minimum physical pan (yaw) angular position
            -18000.0, // Mount2's maximum pan angle, MOunt2's maximum physical pan (yaw) angular position
                 0.0, // Mount2's Roll stabilization lead time, Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate, compensating for servo delay. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.
                 0.0, // Mount2's Pitch stabilization lead time, Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.
                 0.0, // Mount2 Type, Mount Type (None, Servo or MAVLink)
                 0.0, // DataFlash Backend Storage type, 0 for None, 1 for File, 2 for dataflash mavlink, 3 for both file and dataflash
        -3.40282e+38, // Maximum DataFlash File Backend buffer size (in kilobytes), The DataFlash_File backend uses a buffer to store data before writing to the block device.  Raising this value may reduce "gaps" in your SD card logging.  This buffer size may be reduced depending on available memory.  PixHawk requires at least 4 kilobytes.  Maximum value available here is 64 kilobytes.
                 0.0, // Enable logging while disarmed, If LOG_DISARMED is set to 1 then logging will be enabled while disarmed. This can make for very large logfiles but can help a lot when tracking down startup issues
                 0.0, // Enable logging of information needed for Replay, If LOG_REPLAY is set to 1 then the EKF2 state estimator will log detailed information needed for diagnosing problems with the Kalman filter. It is suggested that you also raise LOG_FILE_BUFSIZE to give more buffer space for logging and use a high quality microSD card to ensure no sensor data is lost
                 0.0, // Battery monitoring, Controls enabling monitoring of the battery's voltage and current
                -1.0, // Battery Voltage sensing pin, Setting this to 0 ~ 13 will enable battery voltage sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 13. On the PX4 it should be set to 100. On the Pixhawk powered from the PM connector it should be set to 2.
                -1.0, // Battery Current sensing pin, Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 12. On the PX4 it should be set to 101. On the Pixhawk powered from the PM connector it should be set to 3.
        -3.40282e+38, // Voltage Multiplier, Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick on APM2 or Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX4 using the PX4IO power supply this should be set to 1.
        -3.40282e+38, // Amps per volt, Number of amps that a 1V reading on the current sensor corresponds to. On the APM2 or Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.
        -3.40282e+38, // AMP offset, Voltage offset at zero current on current sensor
        -3.40282e+38, // Battery capacity, Capacity of the battery in mAh when full
        -3.40282e+38, // Maximum allowed power (Watts), If battery wattage (voltage * current) exceeds this value then the system will reduce max throttle (THR_MAX, TKOFF_THR_MAX and THR_MIN for reverse thrust) to satisfy this limit. This helps limit high current to low C rated batteries regardless of battery voltage. The max throttle will slowly grow back to THR_MAX (or TKOFF_THR_MAX ) and THR_MIN if demanding the current max and under the watt max. Use 0 to disable.
                 0.0, // Battery monitoring, Controls enabling monitoring of the battery's voltage and current
                -1.0, // Battery Voltage sensing pin, Setting this to 0 ~ 13 will enable battery voltage sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 13. On the PX4 it should be set to 100. On the Pixhawk powered from the PM connector it should be set to 2.
                -1.0, // Battery Current sensing pin, Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 12. On the PX4 it should be set to 101. On the Pixhawk powered from the PM connector it should be set to 3.
        -3.40282e+38, // Voltage Multiplier, Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick on APM2 or Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX4 using the PX4IO power supply this should be set to 1.
        -3.40282e+38, // Amps per volt, Number of amps that a 1V reading on the current sensor corresponds to. On the APM2 or Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.
        -3.40282e+38, // AMP offset, Voltage offset at zero current on current sensor
        -3.40282e+38, // Battery capacity, Capacity of the battery in mAh when full
        -3.40282e+38, // Maximum allowed current, If battery wattage (voltage * current) exceeds this value then the system will reduce max throttle (THR_MAX, TKOFF_THR_MAX and THR_MIN for reverse thrust) to satisfy this limit. This helps limit high current to low C rated batteries regardless of battery voltage. The max throttle will slowly grow back to THR_MAX (or TKOFF_THR_MAX ) and THR_MIN if demanding the current max and under the watt max. Use 0 to disable.
                 0.0, // Auxiliary pin config, Control assigning of FMU pins to PWM output, timer capture and GPIO. All unassigned pins can be used for GPIO
                 0.0, // Serial 1 flow control, Enable flow control on serial 1 (telemetry 1) on Pixhawk. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup. Note that the PX4v1 does not have hardware flow control pins on this port, so you should leave this disabled.
                 0.0, // Serial 2 flow control, Enable flow control on serial 2 (telemetry 2) on Pixhawk and PX4. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.
                 0.0, // Enable use of safety arming switch, This controls the default state of the safety switch at startup. When set to 1 the safety switch will start in the safe state (flashing) at boot. When set to zero the safety switch will start in the unsafe state (solid) at startup. Note that if a safety switch is fitted the user can still control the safety state after startup using the switch. The safety state can also be controlled in software using a MAVLink message.
                 0.0, //  SBUS output rate, This sets the SBUS output frame rate in Hz
            -32767.0, // User-defined serial number, User-defined serial number of this vehicle, it can be any arbitrary number you want and has no effect on the autopilot
                 0.0, //  Enable use of UAVCAN devices, Enabling this option on a Pixhawk enables UAVCAN devices. Note that this uses about 25k of memory
                 0.0, // Channels to which ignore the safety switch state, A bitmask which controls what channels can move while the safety switch has not been pressed
                -1.0, // Target IMU temperature, This sets the target IMU temperature for boards with controllable IMU heating units. A value of -1 disables heating.
                 0.0, // Board type, This allows selection of a PX4 or VRBRAIN board type. If set to zero then the board type is auto-detected (PX4)
                 0.0, // Sprayer enable/disable, Allows you to enable (1) or disable (0) the sprayer
                 0.0, // Pump speed, Desired pump speed when travelling 1m/s expressed as a percentage
              1000.0, // Spinner rotation speed, Spinner's rotation speed in PWM (a higher rate will disperse the spray over a wider area horizontally)
                 0.0, // Speed minimum, Speed minimum at which we will begin spraying
                 0.0, // Pump speed minimum, Minimum pump speed expressed as a percentage
        -3.40282e+38, // Absolute Pressure, calibrated ground pressure in Pascals
        -3.40282e+38, // ground temperature, calibrated ground temperature in degrees Celsius
        -3.40282e+38, // altitude offset, altitude offset in meters added to barometric altitude. This is used to allow for automatic adjustment of the base barometric altitude by a ground station equipped with a barometer. The value is added to the barometric altitude read by the aircraft. It is automatically reset to 0 when the barometer is calibrated on each reboot or when a preflight calibration is performed.
                 0.0, // Primary barometer, This selects which barometer will be the primary if multiple barometers are found
                 0.0, // GPS type, GPS type
                 0.0, // 2nd GPS type, GPS type of 2nd GPS
                 0.0, // Navigation filter setting, Navigation filter engine setting
                 0.0, // Automatic Switchover Setting, Automatic switchover to GPS reporting best lock
                 0.0, // Minimum Lock Type Accepted for DGPS, Sets the minimum type of differential GPS corrections required before allowing to switch into DGPS mode.
                 0.0, // SBAS Mode, This sets the SBAS (satellite based augmentation system) mode if available on this GPS. If set to 2 then the SBAS mode is not changed in the GPS. Otherwise the GPS will be reconfigured to enable/disable SBAS. Disabling SBAS may be worthwhile in some parts of the world where an SBAS signal is available but the baseline is too long to be useful.
              -100.0, // Minimum elevation, This sets the minimum elevation of satellites above the horizon for them to be used for navigation. Setting this to -100 leaves the minimum elevation set to the GPS modules default.
                 0.0, // Destination for GPS_INJECT_DATA MAVLink packets, The GGS can send raw serial packets to inject data to multiple GPSes.
                   0, // Swift Binary Protocol Logging Mask, Masked with the SBP msg_type field to determine whether SBR1/SBR2 data is logged
                 0.0, // Raw data logging, Enable logging of RXM raw data from uBlox which includes carrier phase and pseudo range information. This allows for post processing of dataflash logs for more precise positioning. Note that this requires a raw capable uBlox such as the 6P or 6T.
                 0.0, // GNSS system configuration, Bitmask for what GNSS system to use on the first GPS (all unchecked or zero to leave GPS as configured)
                 0.0, // Save GPS configuration, Determines whether the configuration for this GPS should be written to non-volatile memory on the GPS. Currently working for UBlox 6 series and above.
                 0.0, // GNSS system configuration, Bitmask for what GNSS system to use on the second GPS (all unchecked or zero to leave GPS as configured)
                 0.0, // Automatic GPS configuration, Controls if the autopilot should automatically configure the GPS based on the parameters and default settings
                 0.0, // Scheduler debug level, Set to non-zero to enable scheduler debug messages. When set to show "Slips" the scheduler will display a message whenever a scheduled task is delayed due to too much CPU load. When set to ShowOverruns the scheduled will display a message whenever a task takes longer than the limit promised in the task table.
                50.0, // Scheduling main loop rate, This controls the rate of the main control loop in Hz. This should only be changed by developers. This only takes effect on restart
                 0.0, // Fence enable/disable, Allows you to enable (1) or disable (0) the fence functionality
                 0.0, // Fence Type, Enabled fence types held as bitmask
                 0.0, // Fence Action, What action should be taken when fence is breached
                10.0, // Fence Maximum Altitude, Maximum altitude allowed before geofence triggers
                30.0, // Circular Fence Radius, Circle fence radius which when breached will cause an RTL
                 1.0, // Fence Margin, Distance that autopilot's should maintain from the fence to avoid a breach
                 1.0, // Fence polygon point total, Number of polygon points saved in eeprom (do not update manually)
                 0.0, // Avoidance control enable/disable, Enabled/disable stopping at fence
        -3.40282e+38, // Rally Total, Number of rally points currently loaded
        -3.40282e+38, // Rally Limit, Maximum distance to rally point. If the closest rally point is more than this number of kilometers from the current position and the home location is closer than any of the rally points from the current position then do RTL to home rather than to the closest rally point. This prevents a leftover rally point from a different airfield being used accidentally. If this is set to 0 then the closest rally point is always used.
                 0.0, // Rally Include Home, Controls if Home is included as a Rally point (i.e. as a safe landing place) for RTL
              -180.0, // Servo 1 Position, Angular location of swash servo #1
              -180.0, // Servo 2 Position, Angular location of swash servo #2
              -180.0, // Servo 3 Position, Angular location of swash servo #3
                 0.0, // Tail Type, Tail type selection.  Simpler yaw controller used if external gyro is selected
                 0.0, // Swash Type, Swash Type Setting - either 3-servo CCPM or H1 Mechanical Mixing
                 0.0, // External Gyro Gain, PWM sent to external gyro on ch7 when tail type is Servo w/ ExtGyro
               -90.0, // Swashplate Phase Angle Compensation, Phase angle correction for rotor head.  If pitching the swash forward induces a roll, this can be correct the problem
               -10.0, // Collective-Yaw Mixing, Feed-forward compensation to automatically add rudder input when collective pitch is increased. Can be positive or negative depending on mechanics.
                 0.0, // Flybar Mode Selector, Flybar present or not.  Affects attitude controller used during ACRO flight mode
                 0.0, // Direct Drive VarPitch Tail ESC speed, Direct Drive VarPitch Tail ESC speed.  Only used when TailType is DirectDrive VarPitch
                 0.0, // External Gyro Gain for ACRO, PWM sent to external gyro on ch7 when tail type is Servo w/ ExtGyro. A value of zero means to use H_GYR_GAIN
                 0.0, // RSC PWM output miniumum, This sets the PWM output on RSC channel for maximum rotor speed
                 0.0, // RSC PWM output maxiumum, This sets the PWM output on RSC channel for miniumum rotor speed
                -1.0, // RSC PWM reversal, This controls reversal of the RSC channel output
                 0.0, // Matrix Yaw Min, Yaw control is given at least this pwm range
                0.25, // Thrust Curve Expo, Motor thrust curve exponent (from 0 for linear to 1.0 for second order curve)
                 0.9, // Motor Spin maximum, Point at which the thrust saturates expressed as a number from 0 to 1 in the entire output range
                 6.0, // Battery voltage compensation maximum voltage, Battery voltage compensation maximum voltage (voltage above this will have no additional scaling effect on thrust).  Recommend 4.4 * cell count, 0 = Disabled
                 6.0, // Battery voltage compensation minimum voltage, Battery voltage compensation minimum voltage (voltage below this will have no additional scaling effect on thrust).  Recommend 3.5 * cell count, 0 = Disabled
                 0.0, // Motor Current Max, Maximum current over which maximum throttle is limited (0 = Disabled)
                 0.0, // Output PWM type, This selects the output PWM type, allowing for normal PWM continuous output or OneShot125
                 0.0, // PWM output miniumum, This sets the min PWM output value that will ever be output to the motors, 0 = use input RC3_MIN
                 0.0, // PWM output maximum, This sets the max PWM value that will ever be output to the motors, 0 = use input RC3_MAX
                 0.0, // Motor Spin minimum, Point at which the thrust starts expressed as a number from 0 to 1 in the entire output range
                 0.0, // Motor Spin armed, Point at which the motors start to spin expressed as a number from 0 to 1 in the entire output range
                 0.0, // Motor Current Max Time Constant, Time constant used to limit the maximum current
                0.25, // Thrust Hover Value, Motor thrust needed to hover expressed as a number from 0 to 1
                 0.0, // Hover Value Learning, Enable/Disable automatic learning of hover throttle
                 1.0, // Roll channel, Roll channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Roll is normally on channel 1, but you can move it to any channel with this parameter.  Reboot is required for changes to take effect.
                 1.0, // Pitch channel, Pitch channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Pitch is normally on channel 2, but you can move it to any channel with this parameter.  Reboot is required for changes to take effect.
                 1.0, // Throttle channel, Throttle channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Throttle is normally on channel 3, but you can move it to any channel with this parameter. Warning APM 2.X: Changing the throttle channel could produce unexpected fail-safe results if connection between receiver and on-board PPM Encoder is lost. Disabling on-board PPM Encoder is recommended.  Reboot is required for changes to take effect.
                 1.0, // Yaw channel, Yaw channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Yaw (also known as rudder) is normally on channel 4, but you can move it to any channel with this parameter.  Reboot is required for changes to take effect.
                 0.0, // Enable EKF1, This enables EKF1 to be disabled when using alternative algorithms. When disabling it, the alternate EKF2 estimator must be enabled by setting EK2_ENABLED = 1 and flight control algorithms must be set to use the alternative estimator by setting AHRS_EKF_TYPE = 2.
                0.05, // GPS horizontal velocity measurement noise scaler, This is the scaler that is applied to the speed accuracy reported by the receiver to estimate the horizontal velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then a speed accuracy of 1 is assumed. Increasing it reduces the weighting on these measurements.
                0.05, // GPS vertical velocity measurement noise scaler, This is the scaler that is applied to the speed accuracy reported by the receiver to estimate the vertical velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then a speed accuracy of 1 is assumed. Increasing it reduces the weighting on this measurement.
                 0.1, // GPS horizontal position measurement noise (m), This is the RMS value of noise in the GPS horizontal position measurements. Increasing it reduces the weighting on these measurements.
                 0.1, // Altitude measurement noise (m), This is the RMS value of noise in the altitude measurement. Increasing it reduces the weighting on this measurement.
                0.01, // Magnetometer measurement noise (Gauss), This is the RMS value of noise in magnetometer measurements. Increasing it reduces the weighting on these measurements.
                 0.5, // Equivalent airspeed measurement noise (m/s), This is the RMS value of noise in equivalent airspeed measurements. Increasing it reduces the weighting on these measurements.
                0.01, // Wind velocity process noise (m/s^2), This noise controls the growth of wind state error estimates. Increasing it makes wind estimation faster and noisier.
                 0.0, // Height rate to wind procss noise scaler, Increasing this parameter increases how rapidly the wind states adapt when changing altitude, but does make wind speed estimation noiser.
               0.001, // Rate gyro noise (rad/s), This noise controls the growth of estimated error due to gyro measurement errors excluding bias. Increasing it makes the flter trust the gyro measurements less and other measurements more.
                0.05, // Accelerometer noise (m/s^2), This noise controls the growth of estimated error due to accelerometer measurement errors excluding bias. Increasing it makes the flter trust the accelerometer measurements less and other measurements more.
               1e-07, // Rate gyro bias process noise (rad/s), This noise controls the growth of gyro bias state error estimates. Increasing it makes rate gyro bias estimation faster and noisier.
               1e-05, // Accelerometer bias process noise (m/s^2), This noise controls the growth of the vertical acelerometer bias state error estimate. Increasing it makes accelerometer bias estimation faster and noisier.
              0.0001, // Earth magnetic field process noise (gauss/s), This noise controls the growth of earth magnetic field state error estimates. Increasing it makes earth magnetic field bias estimation faster and noisier.
              0.0001, // Body magnetic field process noise (gauss/s), This noise controls the growth of body magnetic field state error estimates. Increasing it makes compass offset estimation faster and noisier.
                 0.0, // GPS velocity measurement delay (msec), This is the number of msec that the GPS velocity measurements lag behind the inertial measurements.
                 0.0, // GPS position measurement delay (msec), This is the number of msec that the GPS position measurements lag behind the inertial measurements.
                 0.0, // GPS mode control, This parameter controls use of GPS measurements : 0 = use 3D velocity & 2D position, 1 = use 2D velocity and 2D position, 2 = use 2D position, 3 = use no GPS (optical flow will be used if available)
                 1.0, // GPS velocity measurement gate size, This parameter sets the number of standard deviations applied to the GPS velocity measurement innovation consistency check. Decreasing it makes it more likely that good measurements willbe rejected. Increasing it makes it more likely that bad measurements will be accepted.
                 1.0, // GPS position measurement gate size, This parameter sets the number of standard deviations applied to the GPS position measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
                 1.0, // Height measurement gate size, This parameter sets the number of standard deviations applied to the height measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
                 1.0, // Magnetometer measurement gate size, This parameter sets the number of standard deviations applied to the magnetometer measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
                 1.0, // Airspeed measurement gate size, This parameter sets the number of standard deviations applied to the airspeed measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
                 0.0, // Magnetometer calibration mode, EKF_MAG_CAL = 0 enables calibration based on flying speed and altitude and is the default setting for Plane users. EKF_MAG_CAL = 1 enables calibration based on manoeuvre level and is the default setting for Copter and Rover users. EKF_MAG_CAL = 2 prevents magnetometer calibration regardless of flight condition and is recommended if in-flight magnetometer calibration is unreliable.
               100.0, // GPS glitch accel gate size (cm/s^2), This parameter controls the maximum amount of difference in horizontal acceleration between the value predicted by the filter and the value measured by the GPS before the GPS position data is rejected. If this value is set too low, then valid GPS data will be regularly discarded, and the position accuracy will degrade. If this parameter is set too high, then large GPS glitches will cause large rapid changes in position.
                10.0, // GPS glitch radius gate size (m), This parameter controls the maximum amount of difference in horizontal position (in m) between the value predicted by the filter and the value measured by the GPS before the long term glitch protection logic is activated and the filter states are reset to the new GPS position. Position steps smaller than this value will be temporarily ignored, but will then be accepted and the filter will move to the new position. Position steps larger than this value will be ignored initially, but the filter will then apply an offset to the GPS position measurement.
                 1.0, // Terrain Gradient % RMS, This parameter sets the RMS terrain gradient percentage assumed by the terrain height estimation. Terrain height can be estimated using optical flow and/or range finder sensor data if fitted. Smaller values cause the terrain height estimate to be slower to respond to changes in measurement. Larger values cause the terrain height estimate to be faster to respond, but also more noisy. Generally this value can be reduced if operating over very flat terrain and increased if operating over uneven terrain.
                0.05, // Optical flow measurement noise (rad/s), This is the RMS value of noise and errors in optical flow measurements. Increasing it reduces the weighting on these measurements.
                 1.0, // Optical Flow measurement gate size, This parameter sets the number of standard deviations applied to the optical flow innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
                 0.0, // Optical Flow measurement delay (msec), This is the number of msec that the optical flow measurements lag behind the inertial measurements. It is the time from the end of the optical flow averaging period and does not include the time delay due to the 100msec of averaging within the flow sensor.
                 1.0, // Range finder measurement gate size, This parameter sets the number of standard deviations applied to the range finder innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
                 1.0, // Maximum valid optical flow rate, This parameter sets the magnitude maximum optical flow rate in rad/sec that will be accepted by the filter
                 0.0, // Fallback strictness, This parameter controls the conditions necessary to trigger a fallback to DCM and INAV. A value of 1 will cause fallbacks to occur on loss of GPS and other conditions. A value of 0 will trust the EKF more.
                 0.0, // Primary height source, This parameter controls which height sensor is used by the EKF during optical flow navigation (when EKF_GPS_TYPE = 3). A value of will 0 cause it to always use baro altitude. A value of 1 will cause it to use range finder if available.
        -3.40282e+38, // GPS preflight check, 1 byte bitmap of GPS preflight checks to perform. Set to 0 to bypass all checks. Set to 255 perform all checks. Set to 3 to check just the number of satellites and HDoP. Set to 31 for the most rigorous checks that will still allow checks to pass when the copter is moving, eg launch from a boat.
                 0.0, // Enable EKF2, This enables EKF2. Enabling EKF2 only makes the maths run, it does not mean it will be used for flight control. To use it for flight control set AHRS_EKF_TYPE=2. A reboot or restart will need to be performed after changing the value of EK2_ENABLE for it to take effect.
                 0.0, // GPS mode control, This controls use of GPS measurements : 0 = use 3D velocity & 2D position, 1 = use 2D velocity and 2D position, 2 = use 2D position, 3 = use no GPS (optical flow will be used if available)
                0.05, // GPS horizontal velocity measurement noise (m/s), This sets a lower limit on the speed accuracy reported by the GPS receiver that is used to set horizontal velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then the parameter value will be used. Increasing it reduces the weighting of the GPS horizontal velocity measurements.
                0.05, // GPS vertical velocity measurement noise (m/s), This sets a lower limit on the speed accuracy reported by the GPS receiver that is used to set vertical velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then the parameter value will be used. Increasing it reduces the weighting of the GPS vertical velocity measurements.
               100.0, // GPS velocity innovation gate size, This sets the percentage number of standard deviations applied to the GPS velocity measurement innovation consistency check. Decreasing it makes it more likely that good measurements willbe rejected. Increasing it makes it more likely that bad measurements will be accepted.
                 0.1, // GPS horizontal position measurement noise (m), This sets the GPS horizontal position observation noise. Increasing it reduces the weighting of GPS horizontal position measurements.
               100.0, // GPS position measurement gate size, This sets the percentage number of standard deviations applied to the GPS position measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
                10.0, // GPS glitch radius gate size (m), This controls the maximum radial uncertainty in position between the value predicted by the filter and the value measured by the GPS before the filter position and velocity states are reset to the GPS. Making this value larger allows the filter to ignore larger GPS glitches but also means that non-GPS errors such as IMU and compass can create a larger error in position before the filter is forced back to the GPS position.
                 0.0, // GPS measurement delay (msec), This is the number of msec that the GPS measurements lag behind the inertial measurements.
                 0.0, // Primary height source, This parameter controls the primary height sensor used by the EKF. If the selected option cannot be used, it will default to Baro as the primary height source. Setting 0 will use the baro altitude at all times. Setting 1 uses the range finder and is only available in combination with optical flow navigation (EK2_GPS_TYPE = 3). Setting 2 uses GPS. NOTE - the EK2_RNG_USE_HGT parameter can be used to switch to range-finder when close to the ground.
                 0.1, // Altitude measurement noise (m), This is the RMS value of noise in the altitude measurement. Increasing it reduces the weighting of the baro measurement and will make the filter respond more slowly to baro measurement errors, but will make it more sensitive to GPS and accelerometer errors.
               100.0, // Height measurement gate size, This sets the percentage number of standard deviations applied to the height measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
                 0.0, // Height measurement delay (msec), This is the number of msec that the Height measurements lag behind the inertial measurements.
                0.01, // Magnetometer measurement noise (Gauss), This is the RMS value of noise in magnetometer measurements. Increasing it reduces the weighting on these measurements.
                 0.0, // Magnetometer calibration mode, EKF_MAG_CAL = 0 enables calibration when airborne and is the default setting for Plane users. EKF_MAG_CAL = 1 enables calibration when manoeuvreing. EKF_MAG_CAL = 2 prevents magnetometer calibration regardless of flight condition, is recommended if the external magnetic field is varying and is the default for rovers. EKF_MAG_CAL = 3 enables calibration when the first in-air field and yaw reset has completed and is the default for copters. EKF_MAG_CAL = 4 enables calibration all the time. This determines when the filter will use the 3-axis magnetometer fusion model that estimates both earth and body fixed magnetic field states. This model is only suitable for use when the external magnetic field environment is stable.
               100.0, // Magnetometer measurement gate size, This sets the percentage number of standard deviations applied to the magnetometer measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
                 0.5, // Equivalent airspeed measurement noise (m/s), This is the RMS value of noise in equivalent airspeed measurements used by planes. Increasing it reduces the weighting of airspeed measurements and will make wind speed estimates less noisy and slower to converge. Increasing also increases navigation errors when dead-reckoning without GPS measurements.
               100.0, // Airspeed measurement gate size, This sets the percentage number of standard deviations applied to the airspeed measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
                 0.1, // Range finder measurement noise (m), This is the RMS value of noise in the range finder measurement. Increasing it reduces the weighting on this measurement.
               100.0, // Range finder measurement gate size, This sets the percentage number of standard deviations applied to the range finder innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
                 1.0, // Maximum valid optical flow rate, This sets the magnitude maximum optical flow rate in rad/sec that will be accepted by the filter
                0.05, // Optical flow measurement noise (rad/s), This is the RMS value of noise and errors in optical flow measurements. Increasing it reduces the weighting on these measurements.
               100.0, // Optical Flow measurement gate size, This sets the percentage number of standard deviations applied to the optical flow innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
                 0.0, // Optical Flow measurement delay (msec), This is the number of msec that the optical flow measurements lag behind the inertial measurements. It is the time from the end of the optical flow averaging period and does not include the time delay due to the 100msec of averaging within the flow sensor.
              0.0001, // Rate gyro noise (rad/s), This control disturbance noise controls the growth of estimated error due to gyro measurement errors excluding bias. Increasing it makes the flter trust the gyro measurements less and other measurements more.
                0.01, // Accelerometer noise (m/s^2), This control disturbance noise controls the growth of estimated error due to accelerometer measurement errors excluding bias. Increasing it makes the flter trust the accelerometer measurements less and other measurements more.
               1e-05, // Rate gyro bias stability (rad/s/s), This state  process noise controls growth of the gyro delta angle bias state error estimate. Increasing it makes rate gyro bias estimation faster and noisier.
               1e-06, // Rate gyro scale factor stability (1/s), This noise controls the rate of gyro scale factor learning. Increasing it makes rate gyro scale factor estimation faster and noisier.
               1e-05, // Accelerometer bias stability (m/s^3), This noise controls the growth of the vertical accelerometer delta velocity bias state error estimate. Increasing it makes accelerometer bias estimation faster and noisier.
                0.01, // Wind velocity process noise (m/s^2), This state process noise controls the growth of wind state error estimates. Increasing it makes wind estimation faster and noisier.
                 0.0, // Height rate to wind procss noise scaler, This controls how much the process noise on the wind states is increased when gaining or losing altitude to take into account changes in wind speed and direction with altitude. Increasing this parameter increases how rapidly the wind states adapt when changing altitude, but does make wind velocity estimation noiser.
        -3.40282e+38, // GPS preflight check, This is a 1 byte bitmap controlling which GPS preflight checks are performed. Set to 0 to bypass all checks. Set to 255 perform all checks. Set to 3 to check just the number of satellites and HDoP. Set to 31 for the most rigorous checks that will still allow checks to pass when the copter is moving, eg launch from a boat.
                 1.0, // Bitmask of active IMUs, 1 byte bitmap of IMUs to use in EKF2. A separate instance of EKF2 will be started for each IMU selected. Set to 1 to use the first IMU only (default), set to 2 to use the second IMU only, set to 3 to use the first and second IMU. Additional IMU's can be used up to a maximum of 6 if memory and processing resources permit. There may be insufficient memory and processing resources to run multiple instances. If this occurs EKF2 will fail to start.
                50.0, // GPS accuracy check scaler (%), This scales the thresholds that are used to check GPS accuracy before it is used by the EKF. A value of 100 is the default. Values greater than 100 increase and values less than 100 reduce the maximum GPS error the EKF will accept. A value of 200 will double the allowable GPS error.
                 0.5, // Non-GPS operation position uncertainty (m), This sets the amount of position variation that the EKF allows for when operating without external measurements (eg GPS or optical flow). Increasing this parameter makes the EKF attitude estimate less sensitive to vehicle manoeuvres but more sensitive to IMU errors.
                 0.0, // EKF sensor logging IMU mask, This sets the IMU mask of sensors to do full logging for
                0.05, // Yaw measurement noise (rad), This is the RMS value of noise in yaw measurements from the magnetometer. Increasing it reduces the weighting on these measurements.
               100.0, // Yaw measurement gate size, This sets the percentage number of standard deviations applied to the magnetometer yaw measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
                10.0, // Output complementary filter time constant (centi-sec), Sets the time constant of the output complementary filter/predictor in centi-seconds.
               1e-05, // Earth magnetic field process noise (gauss/s), This state process noise controls the growth of earth magnetic field state error estimates. Increasing it makes earth magnetic field estimation faster and noisier.
               1e-05, // Body magnetic field process noise (gauss/s), This state process noise controls the growth of body magnetic field state error estimates. Increasing it makes magnetometer bias error estimation faster and noisier.
                -1.0, // Range finder switch height percentage, The range finder will be used as the primary height source when below a specified percentage of the sensor maximum as set by the RNGFND_MAX_CM parameter. Set to -1 to prevent range finder use.
                 0.0, // Total mission commands, The number of mission mission items that has been loaded by the ground station. Do not change this manually.
                 0.0, // Mission Restart when entering Auto mode, Controls mission starting point when entering Auto mode (either restart from beginning of mission or resume from last command run)
                 0.0, // RSSI Type, Radio Receiver RSSI type. If your radio receiver supports RSSI of some kind, set it here, then set its associated RSSI_XXXXX parameters, if any.
                 0.0, // Receiver RSSI analog sensing pin, This selects an analog pin where the receiver RSSI voltage will be read.
                 0.0, // Receiver RSSI voltage low, This is the voltage value that the radio receiver will put on the RSSI_ANA_PIN when the signal strength is the weakest. Since some radio receivers put out inverted values from what you might otherwise expect, this isn't necessarily a lower value than RSSI_PIN_HIGH. 
                 0.0, // Receiver RSSI voltage high, This is the voltage value that the radio receiver will put on the RSSI_ANA_PIN when the signal strength is the strongest. Since some radio receivers put out inverted values from what you might otherwise expect, this isn't necessarily a higher value than RSSI_PIN_LOW. 
        -3.40282e+38, // Receiver RSSI channel number, The channel number where RSSI will be output by the radio receiver (5 and above).
                 0.0, // Receiver RSSI PWM low value, This is the PWM value that the radio receiver will put on the RSSI_CHANNEL when the signal strength is the weakest. Since some radio receivers put out inverted values from what you might otherwise expect, this isn't necessarily a lower value than RSSI_CHAN_HIGH. 
                 0.0, // Receiver RSSI PWM high value, This is the PWM value that the radio receiver will put on the RSSI_CHANNEL when the signal strength is the strongest. Since some radio receivers put out inverted values from what you might otherwise expect, this isn't necessarily a higher value than RSSI_CHAN_LOW. 
                 0.0, // Rangefinder type, What type of rangefinder device that is connected
                -1.0, // Rangefinder pin, Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
        -3.40282e+38, // Rangefinder scaling, Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
        -3.40282e+38, // rangefinder offset, Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM and I2C Lidars
                 0.0, // Rangefinder function, Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
        -3.40282e+38, // Rangefinder minimum distance, Minimum distance in centimeters that rangefinder can reliably read
        -3.40282e+38, // Rangefinder maximum distance, Maximum distance in centimeters that rangefinder can reliably read
                -1.0, // Rangefinder stop pin, Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
        -3.40282e+38, // Rangefinder settle time, The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
                 0.0, // Ratiometric, This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
                 0.0, // Powersave range, This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode (if available). A value of zero means power saving is not enabled
                 0.0, // Distance (in cm) from the range finder to the ground, This parameter sets the expected range measurement(in cm) that the range finder should return when the vehicle is on the ground.
                 0.0, // Bus address of sensor, This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
                 0.0, // Second Rangefinder type, What type of rangefinder device that is connected
                -1.0, // Rangefinder pin, Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
        -3.40282e+38, // Rangefinder scaling, Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
        -3.40282e+38, // rangefinder offset, Offset in volts for zero distance
                 0.0, // Rangefinder function, Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
        -3.40282e+38, // Rangefinder minimum distance, Minimum distance in centimeters that rangefinder can reliably read
        -3.40282e+38, // Rangefinder maximum distance, Maximum distance in centimeters that rangefinder can reliably read
                -1.0, // Rangefinder stop pin, Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
        -3.40282e+38, // Sonar settle time, The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
                 0.0, // Ratiometric, This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
                 0.0, // Distance (in cm) from the second range finder to the ground, This parameter sets the expected range measurement(in cm) that the second range finder should return when the vehicle is on the ground.
                 0.0, // Bus address of second rangefinder, This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
                 0.0, // Third Rangefinder type, What type of rangefinder device that is connected
                -1.0, // Rangefinder pin, Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
        -3.40282e+38, // Rangefinder scaling, Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
        -3.40282e+38, // rangefinder offset, Offset in volts for zero distance
                 0.0, // Rangefinder function, Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
        -3.40282e+38, // Rangefinder minimum distance, Minimum distance in centimeters that rangefinder can reliably read
        -3.40282e+38, // Rangefinder maximum distance, Maximum distance in centimeters that rangefinder can reliably read
                -1.0, // Rangefinder stop pin, Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
        -3.40282e+38, // Sonar settle time, The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
                 0.0, // Ratiometric, This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
                 0.0, // Distance (in cm) from the third range finder to the ground, This parameter sets the expected range measurement(in cm) that the third range finder should return when the vehicle is on the ground.
                 0.0, // Bus address of third rangefinder, This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
                 0.0, // Fourth Rangefinder type, What type of rangefinder device that is connected
                -1.0, // Rangefinder pin, Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
        -3.40282e+38, // Rangefinder scaling, Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
        -3.40282e+38, // rangefinder offset, Offset in volts for zero distance
                 0.0, // Rangefinder function, Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
        -3.40282e+38, // Rangefinder minimum distance, Minimum distance in centimeters that rangefinder can reliably read
        -3.40282e+38, // Rangefinder maximum distance, Maximum distance in centimeters that rangefinder can reliably read
                -1.0, // Rangefinder stop pin, Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
        -3.40282e+38, // Sonar settle time, The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
                 0.0, // Ratiometric, This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
                 0.0, // Distance (in cm) from the fourth range finder to the ground, This parameter sets the expected range measurement(in cm) that the fourth range finder should return when the vehicle is on the ground.
                 0.0, // Bus address of fourth rangefinder, This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
                 0.0, // Terrain data enable, enable terrain data. This enables the vehicle storing a database of terrain data on the SD card. The terrain data is requested from the ground station as needed, and stored for later use on the SD card. To be useful the ground station must support TERRAIN_REQUEST messages and have access to a terrain database, such as the SRTM database.
        -3.40282e+38, // Terrain grid spacing, Distance between terrain grid points in meters. This controls the horizontal resolution of the terrain data that is stored on te SD card and requested from the ground station. If your GCS is using the worldwide SRTM database then a resolution of 100 meters is appropriate. Some parts of the world may have higher resolution data available, such as 30 meter data available in the SRTM database in the USA. The grid spacing also controls how much data is kept in memory during flight. A larger grid spacing will allow for a larger amount of data in memory. A grid spacing of 100 meters results in the vehicle keeping 12 grid squares in memory with each grid square having a size of 2.7 kilometers by 3.2 kilometers. Any additional grid squares are stored on the SD once they are fetched from the GCS and will be demand loaded as needed.
                 0.0, // Optical flow enable/disable, Setting this to Enabled(1) will enable optical flow. Setting this to Disabled(0) will disable optical flow
              -200.0, // X axis optical flow scale factor correction, This sets the parts per thousand scale factor correction applied to the flow sensor X axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the X axis optical flow reading by 0.1%. Negative values reduce the scale factor.
              -200.0, // Y axis optical flow scale factor correction, This sets the parts per thousand scale factor correction applied to the flow sensor Y axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the Y axis optical flow reading by 0.1%. Negative values reduce the scale factor.
            -18000.0, // Flow sensor yaw alignment, Specifies the number of centi-degrees that the flow sensor is yawed relative to the vehicle. A sensor with its X-axis pointing to the right of the vehicle X axis has a positive yaw angle.
                 0.0, // Precision Land enabled/disabled and behaviour, Precision Land enabled/disabled and behaviour
                 0.0, // Precision Land Type, Precision Land Type
                 0.0, // RPM type, What type of RPM sensor is connected
        -3.40282e+38, // RPM scaling, Scaling factor between sensor reading and RPM.
        -3.40282e+38, // Maximum RPM, Maximum RPM to report
        -3.40282e+38, // Minimum RPM, Minimum RPM to report
        -3.40282e+38, // Minimum Quality, Minimum data quality to be used
                 0.0, // Second RPM type, What type of RPM sensor is connected
        -3.40282e+38, // RPM scaling, Scaling factor between sensor reading and RPM.
                 0.0, // Enable ADSB, Enable ADS-B
                 1.0, // ADSB vehicle list size, ADSB list size of nearest vehicles. Longer lists take longer to refresh with lower SRx_ADSB values.
                 1.0, // ADSB vehicle list radius filter, ADSB vehicle list radius filter. Vehicles detected outside this radius will be completely ignored. They will not show up in the SRx_ADSB stream to the GCS and will not be considered in any avoidance calculations.
                -1.0, // ICAO_ID vehicle identifaction number, ICAO_ID unique vehicle identifaction number of this aircraft. This is a integer limited to 24bits. If set to 0 then one will be randomly generated. If set to -1 then static information is not sent, transceiver is assumed pre-programmed.
                 0.0, // Emitter type, ADSB classification for the type of vehicle emitting the transponder signal. Default value is 14 (UAV).
                 0.0, // Aircraft length and width, Aircraft length and width dimension options in Length and Width in meters. In most cases, use a value of 1 for smallest size.
                 0.0, // GPS antenna lateral offset, GPS antenna lateral offset. This describes the physical location offest from center of the GPS antenna on the aircraft.
                 0.0, // GPS antenna longitudinal offset, GPS antenna longitudinal offset. This is usually set to 1, Applied By Sensor
                 0.0, // Transceiver RF selection, Transceiver RF selection for Rx enable and/or Tx enable.
                 0.0, // Enable Avoidance using ADSB, Enable Avoidance using ADSB
                 0.0, // Recovery behaviour after a fail event, Determines what the aircraft will do after a fail event is resolved
        -3.40282e+38, // Maximum number of obstacles to track, Maximum number of obstacles to track
        -3.40282e+38, // Time Horizon Warn, Aircraft velocity vectors are multiplied by this time to determine closest approach.  If this results in an approach closer than W_DIST_XY or W_DIST_Z then W_ACTION is undertaken (assuming F_ACTION is not undertaken)
        -3.40282e+38, // Time Horizon Fail, Aircraft velocity vectors are multiplied by this time to determine closest approach.  If this results in an approach closer than F_DIST_XY or F_DIST_Z then F_ACTION is undertaken
        -3.40282e+38, // Distance Warn XY, Closest allowed projected distance before W_ACTION is undertaken
        -3.40282e+38, // Distance Fail XY, Closest allowed projected distance before F_ACTION is undertaken
        -3.40282e+38, // Distance Warn Z, Closest allowed projected distance before BEHAVIOUR_W is undertaken
        -3.40282e+38, // Distance Fail Z, Closest allowed projected distance before BEHAVIOUR_F is undertaken
                 0.0, // LED Brightness, Select the RGB LED brightness level. When USB is connected brightness will never be higher than low regardless of the setting.
                 0.0, // Buzzer enable, Enable or disable the buzzer. Only for Linux and PX4 based boards.
                 0.0, // Setup for MAVLink LED override, This sets up the board RGB LED for override by MAVLink. Normal notify LED control is disabled
                 0.0, // Enable button reporting, This enables the button checking module. When this is disabled the parameters for setting button inputs are not visible
                -1.0, // First button Pin, Digital pin number for first button input. 
                -1.0, // Second button Pin, Digital pin number for second button input. 
                -1.0, // Third button Pin, Digital pin number for third button input. 
                -1.0, // Fourth button Pin, Digital pin number for fourth button input. 
                 0.0, // Report send time, The duration in seconds that a BUTTON_CHANGE report is repeatedly sent to the GCS regarding a button changing state. Note that the BUTTON_CHANGE message is MAVLink2 only.
};
const float param_max[MAVLINK_XCP_PARAMETERS] = { 
         3.40282e+38, // Eeprom format version number, This value is incremented when changes are made to the eeprom format
                20.0, // Software Type, This is used by the ground station to recognise the software type (eg ArduPlane vs ArduCopter)
               255.0, // MAVLink system ID of this vehicle, Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network
               255.0, // My ground station number, Allows restricting radio overrides to only come from my ground station
                 1.0, // CLI Enable, This enables/disables the checking for three carriage returns on telemetry links on startup to enter the diagnostics command line interface
                10.0, // Throttle filter cutoff, Throttle filter cutoff (Hz) - active whenever altitude control is inactive - 0 to disable
              1000.0, // Pilot takeoff altitude, Altitude that altitude control modes will climb to when a takeoff is triggered with the throttle stick.
               500.0, // Takeoff trigger deadzone, Offset from mid stick at which takeoff is triggered
                 4.0, // Throttle stick behavior, Bitmask containing various throttle stick options. Add up the values for options that you want.
                10.0, // Telemetry startup delay, The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
                 4.0, // GCS PID tuning mask, bitmask of PIDs to send MAVLink PID_TUNING messages for
              8000.0, // RTL Altitude, The minimum relative altitude the model will move to before Returning to Launch.  Set to zero to return at current altitude.
                10.0, // RTL cone slope, Defines a cone above home which determines maximum climb
              2000.0, // RTL speed, Defines the speed in cm/s which the aircraft will attempt to maintain horizontally while flying home. If this is set to zero, WPNAV_SPEED will be used instead.
                 2.0, // Rangefinder gain, Used to adjust the speed with which the target altitude is changed when objects are sensed below the copter
                 2.0, // Battery Failsafe Enable, Controls whether failsafe will be invoked when battery voltage or current runs low
         3.40282e+38, // Failsafe battery voltage, Battery voltage to trigger failsafe. Set to 0 to disable battery voltage failsafe. If the battery voltage drops below this voltage then the copter will RTL
         3.40282e+38, // Failsafe battery milliAmpHours, Battery capacity remaining to trigger failsafe. Set to 0 to disable battery remaining failsafe. If the battery remaining drops below this level then the copter will RTL
                 2.0, // Ground Station Failsafe Enable, Controls whether failsafe will be invoked (and what action to take) when connection with Ground station is lost for at least 5 seconds. NB. The GCS Failsafe is only active when RC_OVERRIDE is being used to control the vehicle.
               900.0, // GPS Hdop Good, GPS Hdop value at or below this value represent a good position.  Used for pre-arm checks
                 1.0, // Compass enable/disable, Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass
                63.0, // Super Simple Mode, Bitmask to enable Super Simple mode for some flight modes. Setting this to Disabled(0) will disable Super Simple Mode
              1000.0, // RTL Final Altitude, This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission.  Set to zero to land.
              3000.0, // RTL minimum climb, The vehicle will climb this many cm during the initial climb portion of the RTL
                 3.0, // Yaw behaviour during missions, Determines how the autopilot controls the yaw during missions and RTL
             60000.0, // RTL loiter time, Time (in milliseconds) to loiter above home before beginning final descent
               200.0, // Land speed, The descent speed for the final stage of landing in cm/s
               500.0, // Land speed high, The descent speed for the first stage of landing in cm/s. If this is zero then WPNAV_SPEED_DN is used
               500.0, // Pilot maximum vertical speed, The maximum vertical velocity the pilot may request in cm/s
               500.0, // Pilot vertical acceleration, The vertical acceleration used when pilot is controlling the altitude
                 3.0, // Throttle Failsafe Enable, The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
              1100.0, // Throttle Failsafe Value, The PWM level on channel 3 below which throttle sailsafe triggers
               300.0, // Throttle deadzone, The deadzone above and below mid throttle.  Used in AltHold, Loiter, PosHold flight modes
                20.0, // Flight Mode 1, Flight mode when Channel 5 pwm is <= 1230
                20.0, // Flight Mode 2, Flight mode when Channel 5 pwm is >1230, <= 1360
                20.0, // Flight Mode 3, Flight mode when Channel 5 pwm is >1360, <= 1490
                20.0, // Flight Mode 4, Flight mode when Channel 5 pwm is >1490, <= 1620
                20.0, // Flight Mode 5, Flight mode when Channel 5 pwm is >1620, <= 1749
                20.0, // Flight Mode 6, Flight mode when Channel 5 pwm is >=1750
         3.40282e+38, // Simple mode bitmask, Bitmask which holds which flight modes use simple heading mode (eg bit 0 = 1 means Flight Mode 0 uses simple mode)
            655358.0, // Log bitmask, 4 byte bitmap of log types to enable
                 9.0, // ESC Calibration, Controls whether ArduCopter will enter ESC calibration on the next restart.  Do not adjust this parameter manually.
                54.0, // Channel 6 Tuning, Controls which parameters (normally PID gains) are being tuned with transmitter's channel 6 knob
             32767.0, // Tuning minimum, The minimum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
             32767.0, // Tuning maximum, The maximum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
                10.0, // Frame Orientation (+, X or V), Controls motor mixing for multicopters.  Not used for Tri or Traditional Helicopters.
                38.0, // Channel 7 option, Select which function if performed when CH7 is above 1800 pwm
                38.0, // Channel 8 option, Select which function if performed when CH8 is above 1800 pwm
                38.0, // Channel 9 option, Select which function if performed when CH9 is above 1800 pwm
                38.0, // Channel 10 option, Select which function if performed when CH10 is above 1800 pwm
                38.0, // Channel 11 option, Select which function if performed when CH11 is above 1800 pwm
                38.0, // Channel 12 option, Select which function if performed when CH12 is above 1800 pwm
               127.0, // Arming check, Allows enabling or disabling of pre-arming checks of receiver, accelerometer, barometer, compass and GPS
               127.0, // Disarm delay, Delay before automatic disarm in seconds. A value of zero disables auto disarm.
              8000.0, // Angle Max, Maximum lean angle in all flight modes
               100.0, // RC Feel Roll/Pitch, RC feel for roll/pitch which controls vehicle response to user input with 0 being extremely soft and 100 being crisp
                12.0, // PosHold braking rate, PosHold flight mode's rotation rate during braking in deg/sec
              4500.0, // PosHold braking angle max, PosHold flight mode's max lean angle during braking in centi-degrees
                 1.0, // Land repositioning, Enables user input during LAND mode, the landing phase of RTL, and auto mode landings.
                 3.0, // EKF Failsafe Action, Controls the action that will be taken when an EKF failsafe is invoked
                 1.0, // EKF failsafe variance threshold, Allows setting the maximum acceptable compass and velocity variance
                 1.0, // Crash check enable, This enables automatic crash checking. When enabled the motors will disarm if a crash is detected.
               490.0, // ESC Update Speed, This is the speed in Hertz that your ESCs will receive updates
                10.0, // Acro Roll and Pitch P gain, Converts pilot roll and pitch into a desired rate of rotation in ACRO and SPORT mode.  Higher values mean faster rate of rotation.
                10.0, // Acro Yaw P gain, Converts pilot yaw input into a desired rate of rotation in ACRO, Stabilize and SPORT modes.  Higher values mean faster rate of rotation.
                 3.0, // Acro Balance Roll, rate at which roll angle returns to level in acro mode.  A higher value causes the vehicle to return to level faster.
                 3.0, // Acro Balance Pitch, rate at which pitch angle returns to level in acro mode.  A higher value causes the vehicle to return to level faster.
                 2.0, // Acro Trainer, Type of trainer used in acro mode
                 0.5, // Acro Expo, Acro roll/pitch Expo to allow faster rotation when stick at edges
                 6.0, // Velocity (horizontal) P gain, Velocity (horizontal) P gain.  Converts the difference between desired velocity to a target acceleration
                 1.0, // Velocity (horizontal) I gain, Velocity (horizontal) I gain.  Corrects long-term difference in desired velocity to a target acceleration
              4500.0, // Velocity (horizontal) integrator maximum, Velocity (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output
                 8.0, // Velocity (vertical) P gain, Velocity (vertical) P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
                 1.5, // Throttle acceleration controller P gain, Throttle acceleration controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
                 3.0, // Throttle acceleration controller I gain, Throttle acceleration controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
              1000.0, // Throttle acceleration controller I gain maximum, Throttle acceleration controller I gain maximum.  Constrains the maximum pwm that the I term will generate
                 0.4, // Throttle acceleration controller D gain, Throttle acceleration controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
               100.0, // Throttle acceleration filter, Filter applied to acceleration to reduce noise.  Lower values reduce noise but add delay.
                 3.0, // Position (vertical) controller P gain, Position (vertical) controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
                 2.0, // Position (horizonal) controller P gain, Loiter position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller
                 7.0, // Autotune axis bitmask, 1-byte bitmap of axes to autotune
                 0.1, // Autotune aggressiveness, Autotune aggressiveness. Defines the bounce back used to detect size of the D term.
               0.006, // AutoTune minimum D, Defines the minimum D gain
                 1.0, // Start motors before throwing is detected, Used by THROW mode. Controls whether motors will run at the speed set by THR_MIN or will be stopped when armed and waiting for the throw.
                 1.0, // Terrain Following use control, This enables terrain following for RTL and LAND flight modes. To use this option TERRAIN_ENABLE must be 1 and the GCS must  support sending terrain data to the aircraft.  In RTL the RTL_ALT will be considered a height above the terrain.  In LAND mode the vehicle will slow to LAND_SPEED 10m above terrain (instead of 10m above home).  This parameter does not affect AUTO and Guided which use a per-command flag to determine if the height is above-home, absolute or above-terrain.
                 5.0, // Takeoff navigation altitude, This is the altitude in meters above the takeoff point that attitude changes for navigation can begin
                18.0, // Throw mode's follow up mode, Vehicle will switch to this mode after the throw is successfully completed.  Default is to stay in throw mode (18)
                 1.0, // Type of Type, Used by THROW mode. Specifies whether Copter is thrown upward or dropped.
                 1.0, // Ground Effect Compensation Enable/Disable, Ground Effect Compensation Enable/Disable
              1500.0, // Serial0 baud rate, The baud rate used on the USB console. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
                 2.0, // Console protocol selection, Control what protocol to use on the console. 
                 9.0, // Telem1 protocol selection, Control what protocol to use on the Telem1 port. Note that the Frsky options require external converter hardware. See the wiki for details.
              1500.0, // Telem1 Baud Rate, The baud rate used on the Telem1 port. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
                 9.0, // Telemetry 2 protocol selection, Control what protocol to use on the Telem2 port. Note that the Frsky options require external converter hardware. See the wiki for details.
              1500.0, // Telemetry 2 Baud Rate, The baud rate of the Telem2 port. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
                 9.0, // Serial 3 (GPS) protocol selection, Control what protocol Serial 3 (GPS) should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.
              1500.0, // Serial 3 (GPS) Baud Rate, The baud rate used for the Serial 3 (GPS). The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
                 9.0, // Serial4 protocol selection, Control what protocol Serial4 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.
              1500.0, // Serial 4 Baud Rate, The baud rate used for Serial4. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
                 9.0, // Serial5 protocol selection, Control what protocol Serial5 port should be used for. Note that the Frsky options require external converter hardware. See the wiki for details.
              1500.0, // Serial 5 Baud Rate, The baud rate used for Serial5. The APM2 can support all baudrates up to 115, and also can support 500. The PX4 can support rates of up to 1500. If you setup a rate you cannot support on APM2 and then can't connect to your board you should load a firmware from a different vehicle type. That will reset all your parameters to defaults.
              2200.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                 1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
               200.0, // RC dead-zone, dead zone around trim or bottom
              2200.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                 1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
               200.0, // RC dead-zone, dead zone around trim or bottom
              2200.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                 1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
               200.0, // RC dead-zone, dead zone around trim or bottom
              2200.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                 1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
               200.0, // RC dead-zone, dead zone around trim or bottom
              2200.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                 1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
               200.0, // RC dead-zone, dead zone around trim or bottom
                66.0, // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
              2200.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                 1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
               200.0, // RC dead-zone, dead zone around trim or bottom
                66.0, // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
              2200.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                 1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
               200.0, // RC dead-zone, dead zone around trim or bottom
                66.0, // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
              2200.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                 1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
               200.0, // RC dead-zone, dead zone around trim or bottom
                66.0, // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
              2200.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                 1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
               200.0, // RC dead-zone, dead zone around trim or bottom
                66.0, // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
              2200.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                 1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
               200.0, // RC dead-zone, dead zone around trim or bottom
                66.0, // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
              2200.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                 1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
               200.0, // RC dead-zone, dead zone around trim or bottom
                66.0, // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
              2200.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                 1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
               200.0, // RC dead-zone, dead zone around trim or bottom
                66.0, // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
              2200.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                 1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
               200.0, // RC dead-zone, dead zone around trim or bottom
                66.0, // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
              2200.0, // RC min PWM, RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC trim PWM, RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
              2200.0, // RC max PWM, RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
                 1.0, // RC reverse, Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
               200.0, // RC dead-zone, dead zone around trim or bottom
                66.0, // Servo out function, Setting this to Disabled(0) will setup this output for control by auto missions or MAVLink servo set commands. any other value will enable the corresponding function
                 1.0, // Camera shutter (trigger) type, how to trigger the camera to take a picture
                50.0, // Duration that shutter is held open, How long the shutter will be held open in 10ths of a second (i.e. enter 10 for 1second, 50 for 5seconds)
              2000.0, // Servo ON PWM value, PWM value to move servo to when shutter is activated
              2000.0, // Servo OFF PWM value, PWM value to move servo to when shutter is deactivated
              1000.0, // Camera trigger distance, Distance in meters between camera triggers. If this value is non-zero then the camera will trigger whenever the GPS position changes by this number of meters regardless of what mode the APM is in. Note that this parameter can also be set in an auto mission using the DO_SET_CAM_TRIGG_DIST command, allowing you to enable/disable the triggering of the camera during the flight.
                 1.0, // Relay ON value, This sets whether the relay goes high or low when it triggers. Note that you should also set RELAY_DEFAULT appropriately for your camera
             10000.0, // Minimum time between photos, Postpone shooting if previous picture was taken less than preset time(ms) ago.
               180.0, // Maximum photo roll angle., Postpone shooting if roll is greater than limit. (0=Disable, will shoot regardless of roll).
                55.0, // Camera feedback pin, pin number to use for save accurate camera feedback messages. If set to -1 then don't use a pin flag for this, otherwise this is a pin number which if held high after a picture trigger order, will save camera messages when camera really takes a picture. A universal camera hot shoe is needed. The pin should be held high for at least 2 milliseconds for reliable trigger detection. See also the CAM_FEEDBACK_POL option. If using AUX4 pin on a Pixhawk then a fast capture method is used that allows for the trigger time to be as short as one microsecond.
                 1.0, // Camera feedback pin polarity, Polarity for feedback pin. If this is 1 then the feedback pin should go high on trigger. If set to 0 then it should go low
               116.0, // First Relay Pin, Digital pin number for first relay control. This is the pin used for camera control.
               116.0, // Second Relay Pin, Digital pin number for 2nd relay control.
               116.0, // Third Relay Pin, Digital pin number for 3rd relay control.
               116.0, // Fourth Relay Pin, Digital pin number for 4th relay control.
                 2.0, // Default relay state, The state of the relay on boot. 
                 1.0, // EPM Enable/Disable, EPM enable/disable
              2000.0, // EPM Grab PWM, PWM value sent to EPM to initiate grabbing the cargo
              2000.0, // EPM Release PWM, PWM value sent to EPM to release the cargo
              2000.0, // EPM Neutral PWM, PWM value sent to EPM when not grabbing or releasing
               255.0, // EPM UAVCAN Hardpoint ID, Refer to https://docs.zubax.com/opengrab_epm_v3#UAVCAN_interface
                 1.0, // Parachute release enabled or disabled, Parachute release enabled or disabled
                10.0, // Parachute release mechanism type (relay or servo), Parachute release mechanism type (relay or servo)
              2000.0, // Parachute Servo ON PWM value, Parachute Servo PWM value when parachute is released
              2000.0, // Servo OFF PWM value, Parachute Servo PWM value when parachute is not released
             32000.0, // Parachute min altitude in meters above home, Parachute min altitude above home.  Parachute will not be released below this altitude.  0 to disable alt check.
              5000.0, // Parachute release delay, Delay in millseconds between motor stop and chute release
              2000.0, // Landing Gear Servo Retracted PWM Value, Servo PWM value when landing gear is retracted
              2000.0, // Landing Gear Servo Deployed PWM Value, Servo PWM value when landing gear is deployed
               500.0, // Stabilize Mode Collective Point 1, Helicopter's minimum collective pitch setting at zero throttle input in Stabilize mode
               500.0, // Stabilize Mode Collective Point 2, Helicopter's collective pitch setting at mid-low throttle input in Stabilize mode
              1000.0, // Stabilize Mode Collective Point 3, Helicopter's collective pitch setting at mid-high throttle input in Stabilize mode
              1000.0, // Stabilize Mode Collective Point 4, Helicopter's maximum collective pitch setting at full throttle input in Stabilize mode
                 0.5, // Acro Mode Collective Expo, Used to soften collective pitch inputs near center point in Acro mode.
               400.0, // Compass offsets in milligauss on the X axis, Offset to be added to the compass x-axis values to compensate for metal in the frame
               400.0, // Compass offsets in milligauss on the Y axis, Offset to be added to the compass y-axis values to compensate for metal in the frame
               400.0, // Compass offsets in milligauss on the Z axis, Offset to be added to the compass z-axis values to compensate for metal in the frame
               3.142, // Compass declination, An angle to compensate between the true north and magnetic north
                 2.0, // Learn compass offsets automatically, Enable or disable the automatic learning of compass offsets. You can enable learning either using a compass-only method that is suitable only for fixed wing aircraft or using the offsets learnt by the active EKF state estimator. If this option is enabled then the learnt offsets are saved when you disarm the vehicle.
                 1.0, // Use compass for yaw, Enable or disable the use of the compass (instead of the GPS) for determining heading
                 1.0, // Auto Declination, Enable or disable the automatic calculation of the declination based on gps location
                 2.0, // Motor interference compensation type, Set motor interference compensation type to disabled, throttle or current.  Do not change manually.
              1000.0, // Motor interference compensation for body frame X axis, Multiplied by the current throttle and added to the compass's x-axis values to compensate for motor interference
              1000.0, // Motor interference compensation for body frame Y axis, Multiplied by the current throttle and added to the compass's y-axis values to compensate for motor interference
              1000.0, // Motor interference compensation for body frame Z axis, Multiplied by the current throttle and added to the compass's z-axis values to compensate for motor interference
                38.0, // Compass orientation, The orientation of the compass relative to the autopilot board. This will default to the right value for each board type, but can be changed if you have an external compass. See the documentation for your external compass for the right value. The correct orientation should give the X axis forward, the Y axis to the right and the Z axis down. So if your aircraft is pointing west it should show a positive value for the Y axis, and a value close to zero for the X axis. On a PX4 or Pixhawk with an external compass the correct value is zero if the compass is correctly oriented. NOTE: This orientation is combined with any AHRS_ORIENTATION setting.
                 2.0, // Compass is attached via an external cable, Configure compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. Set to 1 if the compass is externally connected. When externally connected the COMPASS_ORIENT option operates independently of the AHRS_ORIENTATION board orientation option. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
               400.0, // Compass2 offsets in milligauss on the X axis, Offset to be added to compass2's x-axis values to compensate for metal in the frame
               400.0, // Compass2 offsets in milligauss on the Y axis, Offset to be added to compass2's y-axis values to compensate for metal in the frame
               400.0, // Compass2 offsets in milligauss on the Z axis, Offset to be added to compass2's z-axis values to compensate for metal in the frame
              1000.0, // Motor interference compensation to compass2 for body frame X axis, Multiplied by the current throttle and added to compass2's x-axis values to compensate for motor interference
              1000.0, // Motor interference compensation to compass2 for body frame Y axis, Multiplied by the current throttle and added to compass2's y-axis values to compensate for motor interference
              1000.0, // Motor interference compensation to compass2 for body frame Z axis, Multiplied by the current throttle and added to compass2's z-axis values to compensate for motor interference
                 2.0, // Choose primary compass, If more than one compass is available this selects which compass is the primary. Normally 0=External, 1=Internal. If no External compass is attached this parameter is ignored
               400.0, // Compass3 offsets in milligauss on the X axis, Offset to be added to compass3's x-axis values to compensate for metal in the frame
               400.0, // Compass3 offsets in milligauss on the Y axis, Offset to be added to compass3's y-axis values to compensate for metal in the frame
               400.0, // Compass3 offsets in milligauss on the Z axis, Offset to be added to compass3's z-axis values to compensate for metal in the frame
              1000.0, // Motor interference compensation to compass3 for body frame X axis, Multiplied by the current throttle and added to compass3's x-axis values to compensate for motor interference
              1000.0, // Motor interference compensation to compass3 for body frame Y axis, Multiplied by the current throttle and added to compass3's y-axis values to compensate for motor interference
              1000.0, // Motor interference compensation to compass3 for body frame Z axis, Multiplied by the current throttle and added to compass3's z-axis values to compensate for motor interference
         3.40282e+38, // Compass device id, Compass device id.  Automatically detected, do not set manually
         3.40282e+38, // Compass2 device id, Second compass's device id.  Automatically detected, do not set manually
         3.40282e+38, // Compass3 device id, Third compass's device id.  Automatically detected, do not set manually
                 1.0, // Compass2 used for yaw, Enable or disable the second compass for determining heading.
                38.0, // Compass2 orientation, The orientation of the second compass relative to the frame (if external) or autopilot board (if internal).
                 2.0, // Compass2 is attached via an external cable, Configure second compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
                 1.0, // Compass3 used for yaw, Enable or disable the third compass for determining heading.
                38.0, // Compass3 orientation, The orientation of the third compass relative to the frame (if external) or autopilot board (if internal).
                 2.0, // Compass3 is attached via an external cable, Configure third compass so it is attached externally. This is auto-detected on PX4 and Pixhawk. If set to 0 or 1 then auto-detection by bus connection can override the value. If set to 2 then auto-detection will be disabled.
         3.40282e+38, // Compass soft-iron diagonal X component, DIA_X in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
         3.40282e+38, // Compass soft-iron diagonal Y component, DIA_Y in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
         3.40282e+38, // Compass soft-iron diagonal Z component, DIA_Z in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
         3.40282e+38, // Compass soft-iron off-diagonal X component, ODI_X in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
         3.40282e+38, // Compass soft-iron off-diagonal Y component, ODI_Y in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
         3.40282e+38, // Compass soft-iron off-diagonal Z component, ODI_Z in the compass soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
         3.40282e+38, // Compass2 soft-iron diagonal X component, DIA_X in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
         3.40282e+38, // Compass2 soft-iron diagonal Y component, DIA_Y in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
         3.40282e+38, // Compass2 soft-iron diagonal Z component, DIA_Z in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
         3.40282e+38, // Compass2 soft-iron off-diagonal X component, ODI_X in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
         3.40282e+38, // Compass2 soft-iron off-diagonal Y component, ODI_Y in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
         3.40282e+38, // Compass2 soft-iron off-diagonal Z component, ODI_Z in the compass2 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
         3.40282e+38, // Compass3 soft-iron diagonal X component, DIA_X in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
         3.40282e+38, // Compass3 soft-iron diagonal Y component, DIA_Y in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
         3.40282e+38, // Compass3 soft-iron diagonal Z component, DIA_Z in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
         3.40282e+38, // Compass3 soft-iron off-diagonal X component, ODI_X in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
         3.40282e+38, // Compass3 soft-iron off-diagonal Y component, ODI_Y in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
         3.40282e+38, // Compass3 soft-iron off-diagonal Z component, ODI_Z in the compass3 soft-iron calibration matrix: [[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]
                20.0, // Compass calibration fitness, This controls the fitness level required for a successful compass calibration. A lower value makes for a stricter fit (less likely to pass). This is the value used for the primary magnetometer. Other magnetometers get double the value.
               257.0, // IMU Product ID, Which type of IMU is installed (read-only).
         3.40282e+38, // Gyro offsets of X axis, Gyro sensor offsets of X axis. This is setup on each boot during gyro calibrations
         3.40282e+38, // Gyro offsets of Y axis, Gyro sensor offsets of Y axis. This is setup on each boot during gyro calibrations
         3.40282e+38, // Gyro offsets of Z axis, Gyro sensor offsets of Z axis. This is setup on each boot during gyro calibrations
         3.40282e+38, // Gyro2 offsets of X axis, Gyro2 sensor offsets of X axis. This is setup on each boot during gyro calibrations
         3.40282e+38, // Gyro2 offsets of Y axis, Gyro2 sensor offsets of Y axis. This is setup on each boot during gyro calibrations
         3.40282e+38, // Gyro2 offsets of Z axis, Gyro2 sensor offsets of Z axis. This is setup on each boot during gyro calibrations
         3.40282e+38, // Gyro3 offsets of X axis, Gyro3 sensor offsets of X axis. This is setup on each boot during gyro calibrations
         3.40282e+38, // Gyro3 offsets of Y axis, Gyro3 sensor offsets of Y axis. This is setup on each boot during gyro calibrations
         3.40282e+38, // Gyro3 offsets of Z axis, Gyro3 sensor offsets of Z axis. This is setup on each boot during gyro calibrations
                 1.2, // Accelerometer scaling of X axis, Accelerometer scaling of X axis.  Calculated during acceleration calibration routine
                 1.2, // Accelerometer scaling of Y axis, Accelerometer scaling of Y axis  Calculated during acceleration calibration routine
                 1.2, // Accelerometer scaling of Z axis, Accelerometer scaling of Z axis  Calculated during acceleration calibration routine
                 3.5, // Accelerometer offsets of X axis, Accelerometer offsets of X axis. This is setup using the acceleration calibration or level operations
                 3.5, // Accelerometer offsets of Y axis, Accelerometer offsets of Y axis. This is setup using the acceleration calibration or level operations
                 3.5, // Accelerometer offsets of Z axis, Accelerometer offsets of Z axis. This is setup using the acceleration calibration or level operations
                 1.2, // Accelerometer2 scaling of X axis, Accelerometer2 scaling of X axis.  Calculated during acceleration calibration routine
                 1.2, // Accelerometer2 scaling of Y axis, Accelerometer2 scaling of Y axis  Calculated during acceleration calibration routine
                 1.2, // Accelerometer2 scaling of Z axis, Accelerometer2 scaling of Z axis  Calculated during acceleration calibration routine
                 3.5, // Accelerometer2 offsets of X axis, Accelerometer2 offsets of X axis. This is setup using the acceleration calibration or level operations
                 3.5, // Accelerometer2 offsets of Y axis, Accelerometer2 offsets of Y axis. This is setup using the acceleration calibration or level operations
                 3.5, // Accelerometer2 offsets of Z axis, Accelerometer2 offsets of Z axis. This is setup using the acceleration calibration or level operations
                 1.2, // Accelerometer3 scaling of X axis, Accelerometer3 scaling of X axis.  Calculated during acceleration calibration routine
                 1.2, // Accelerometer3 scaling of Y axis, Accelerometer3 scaling of Y axis  Calculated during acceleration calibration routine
                 1.2, // Accelerometer3 scaling of Z axis, Accelerometer3 scaling of Z axis  Calculated during acceleration calibration routine
                 3.5, // Accelerometer3 offsets of X axis, Accelerometer3 offsets of X axis. This is setup using the acceleration calibration or level operations
                 3.5, // Accelerometer3 offsets of Y axis, Accelerometer3 offsets of Y axis. This is setup using the acceleration calibration or level operations
                 3.5, // Accelerometer3 offsets of Z axis, Accelerometer3 offsets of Z axis. This is setup using the acceleration calibration or level operations
               127.0, // Gyro filter cutoff frequency, Filter cutoff frequency for gyroscopes. This can be set to a lower value to try to cope with very high vibration levels in aircraft. This option takes effect on the next reboot. A value of zero means no filtering (not recommended!)
               127.0, // Accel filter cutoff frequency, Filter cutoff frequency for accelerometers. This can be set to a lower value to try to cope with very high vibration levels in aircraft. This option takes effect on the next reboot. A value of zero means no filtering (not recommended!)
                 1.0, // Use first IMU for attitude, velocity and position estimates, Use first IMU for attitude, velocity and position estimates
                 1.0, // Use second IMU for attitude, velocity and position estimates, Use second IMU for attitude, velocity and position estimates
                 1.0, // Use third IMU for attitude, velocity and position estimates, Use third IMU for attitude, velocity and position estimates
                50.0, // Stillness threshold for detecting if we are moving, Threshold to tolerate vibration to determine if vehicle is motionless. This depends on the frame type and if there is a constant vibration due to motors before launch or after landing. Total motionless is about 0.05. Suggested values: Planes/rover use 0.1, multirotors use 1, tradHeli uses 5
                 1.0, // Gyro Calibration scheme, Conrols when automatic gyro calibration is performed
                 2.0, // Accel cal trim option, Specifies how the accel cal routine determines the trims
                 3.0, // Body-fixed accelerometer, The body-fixed accelerometer to be used for trim calculation
              2000.0, // Waypoint Horizontal Speed Target, Defines the speed in cm/s which the aircraft will attempt to maintain horizontally during a WP mission
              1000.0, // Waypoint Radius, Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
              1000.0, // Waypoint Climb Speed Target, Defines the speed in cm/s which the aircraft will attempt to maintain while climbing during a WP mission
               500.0, // Waypoint Descent Speed Target, Defines the speed in cm/s which the aircraft will attempt to maintain while descending during a WP mission
              2000.0, // Loiter Horizontal Maximum Speed, Defines the maximum speed in cm/s which the aircraft will travel horizontally while in loiter mode
               500.0, // Waypoint Acceleration , Defines the horizontal acceleration in cm/s/s used during missions
               500.0, // Waypoint Vertical Acceleration, Defines the vertical acceleration in cm/s/s used during missions
              5000.0, // Loiter maximum jerk, Loiter maximum jerk in cm/s/s/s
               981.0, // Loiter maximum acceleration, Loiter maximum acceleration in cm/s/s.  Higher values cause the copter to accelerate and stop more quickly.
               250.0, // Loiter minimum acceleration, Loiter minimum acceleration in cm/s/s. Higher values stop the copter more quickly when the stick is centered, but cause a larger jerk when the copter stops.
             10000.0, // Circle Radius, Defines the radius of the circle the vehicle will fly when in Circle flight mode
                90.0, // Circle rate, Circle mode's turn rate in deg/sec.  Positive to turn clockwise, negative for counter clockwise
             18000.0, // Yaw target slew rate, Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes
             72000.0, // Acceleration Max for Yaw, Maximum acceleration in yaw axis
                 1.0, // Rate Feedforward Enable, Controls whether body-frame rate feedfoward is enabled or disabled
            180000.0, // Acceleration Max for Roll, Maximum acceleration in roll axis
            180000.0, // Acceleration Max for Pitch, Maximum acceleration in pitch axis
                 1.0, // Angle Boost, Angle Boost increases output throttle as the vehicle leans to reduce loss of altitude
                12.0, // Roll axis angle controller P gain, Roll axis angle controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate
                12.0, // Pitch axis angle controller P gain, Pitch axis angle controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
                 6.0, // Yaw axis angle controller P gain, Yaw axis angle controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate
                10.0, // Angle Limit (to maintain altitude) Time Constant, Angle Limit (to maintain altitude) Time Constant
                 0.3, // Roll axis rate controller P gain, Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
                 0.5, // Roll axis rate controller I gain, Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
                 1.0, // Roll axis rate controller I gain maximum, Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
                0.02, // Roll axis rate controller D gain, Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
               100.0, // Roll axis rate conroller input frequency in Hz, Roll axis rate conroller input frequency in Hz
                 0.3, // Pitch axis rate controller P gain, Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output
                 0.5, // Pitch axis rate controller I gain, Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
                 1.0, // Pitch axis rate controller I gain maximum, Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
                0.02, // Pitch axis rate controller D gain, Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
               100.0, // Pitch axis rate conroller input frequency in Hz, Pitch axis rate conroller input frequency in Hz
                 0.5, // Yaw axis rate controller P gain, Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
                0.05, // Yaw axis rate controller I gain, Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
                 1.0, // Yaw axis rate controller I gain maximum, Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
                0.02, // Yaw axis rate controller D gain, Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
               100.0, // Yaw axis rate conroller input frequency in Hz, Yaw axis rate conroller input frequency in Hz
                0.25, // Throttle Mix Minimum, Throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
                 0.9, // Throttle Mix Maximum, Throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
                 5.0, // XY Acceleration filter cutoff frequency, Lower values will slow the response of the navigation controller and reduce twitchiness
                10.0, // Raw sensor stream rate, Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
                10.0, // Extended status stream rate to ground station, Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
                10.0, // RC Channel stream rate to ground station, Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS_RAW to ground station
                10.0, // Raw Control stream rate to ground station, Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station
                10.0, // Position stream rate to ground station, Stream rate of GLOBAL_POSITION_INT to ground station
                10.0, // Extra data type 1 stream rate to ground station, Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station
                10.0, // Extra data type 2 stream rate to ground station, Stream rate of VFR_HUD to ground station
                10.0, // Extra data type 3 stream rate to ground station, Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
                10.0, // Parameter stream rate to ground station, Stream rate of PARAM_VALUE to ground station
                50.0, // ADSB stream rate to ground station, ADSB stream rate to ground station
                10.0, // Raw sensor stream rate, Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
                10.0, // Extended status stream rate to ground station, Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
                10.0, // RC Channel stream rate to ground station, Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS_RAW to ground station
                10.0, // Raw Control stream rate to ground station, Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station
                10.0, // Position stream rate to ground station, Stream rate of GLOBAL_POSITION_INT to ground station
                10.0, // Extra data type 1 stream rate to ground station, Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station
                10.0, // Extra data type 2 stream rate to ground station, Stream rate of VFR_HUD to ground station
                10.0, // Extra data type 3 stream rate to ground station, Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
                10.0, // Parameter stream rate to ground station, Stream rate of PARAM_VALUE to ground station
                50.0, // ADSB stream rate to ground station, ADSB stream rate to ground station
                10.0, // Raw sensor stream rate, Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
                10.0, // Extended status stream rate to ground station, Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
                10.0, // RC Channel stream rate to ground station, Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS_RAW to ground station
                10.0, // Raw Control stream rate to ground station, Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station
                10.0, // Position stream rate to ground station, Stream rate of GLOBAL_POSITION_INT to ground station
                10.0, // Extra data type 1 stream rate to ground station, Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station
                10.0, // Extra data type 2 stream rate to ground station, Stream rate of VFR_HUD to ground station
                10.0, // Extra data type 3 stream rate to ground station, Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
                10.0, // Parameter stream rate to ground station, Stream rate of PARAM_VALUE to ground station
                50.0, // ADSB stream rate to ground station, ADSB stream rate to ground station
                10.0, // Raw sensor stream rate, Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
                10.0, // Extended status stream rate to ground station, Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
                10.0, // RC Channel stream rate to ground station, Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS_RAW to ground station
                10.0, // Raw Control stream rate to ground station, Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station
                10.0, // Position stream rate to ground station, Stream rate of GLOBAL_POSITION_INT to ground station
                10.0, // Extra data type 1 stream rate to ground station, Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station
                10.0, // Extra data type 2 stream rate to ground station, Stream rate of VFR_HUD to ground station
                10.0, // Extra data type 3 stream rate to ground station, Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
                10.0, // Parameter stream rate to ground station, Stream rate of PARAM_VALUE to ground station
                50.0, // ADSB stream rate to ground station, ADSB stream rate to ground station
                 1.0, // AHRS GPS gain, This controls how how much to use the GPS to correct the attitude. This should never be set to zero for a plane as it would result in the plane losing control in turns. For a plane please use the default value of 1.0.
                 1.0, // AHRS use GPS for navigation, This controls whether to use dead-reckoning or GPS based navigation. If set to 0 then the GPS won't be used for navigation, and only dead reckoning will be used. A value of zero should never be used for normal flight.
                 0.4, // Yaw P, This controls the weight the compass or GPS has on the heading. A higher value means the heading will track the yaw source (GPS or compass) more rapidly.
                 0.4, // AHRS RP_P, This controls how fast the accelerometers correct the attitude
               127.0, // Maximum wind, This sets the maximum allowable difference between ground speed and airspeed. This allows the plane to cope with a failing airspeed sensor. A value of zero means to use the airspeed as is.
              0.1745, // AHRS Trim Roll, Compensates for the roll angle difference between the control board and the frame. Positive values make the vehicle roll right.
              0.1745, // AHRS Trim Pitch, Compensates for the pitch angle difference between the control board and the frame. Positive values make the vehicle pitch up/back.
              0.1745, // AHRS Trim Yaw, Not Used
                37.0, // Board Orientation, Overall board orientation relative to the standard orientation for the board type. This rotates the IMU and compass readings to allow the board to be oriented in your vehicle at any 90 or 45 degree angle. This option takes affect on next boot. After changing you will need to re-level your vehicle.
                 0.5, // AHRS Velocity Complementary Filter Beta Coefficient, This controls the time constant for the cross-over frequency used to fuse AHRS (airspeed and heading) and GPS data to estimate ground velocity. Time constant is 0.1/beta. A larger time constant will use GPS data less and a small time constant will use air data less.
                10.0, // AHRS GPS Minimum satellites, Minimum number of satellites visible to use GPS for velocity based corrections attitude correction. This defaults to 6, which is about the point at which the velocity numbers from a GPS become too unreliable for accurate correction of the accelerometers.
                 2.0, // Use NavEKF Kalman filter for attitude and position estimation, This controls whether the NavEKF Kalman filter is used for attitude and position estimation and whether fallback to the DCM algorithm is allowed. Note that on copters "disabled" is not available, and will be the same as "enabled - no fallback"
                 4.0, // Mount default operating mode, Mount default operating mode on startup and after control is returned from autopilot
              179.99, // Mount roll angle when in retracted position, Mount roll angle when in retracted position
              179.99, // Mount tilt/pitch angle when in retracted position, Mount tilt/pitch angle when in retracted position
              179.99, // Mount yaw/pan angle when in retracted position, Mount yaw/pan angle when in retracted position
              179.99, // Mount roll angle when in neutral position, Mount roll angle when in neutral position
              179.99, // Mount tilt/pitch angle when in neutral position, Mount tilt/pitch angle when in neutral position
              179.99, // Mount pan/yaw angle when in neutral position, Mount pan/yaw angle when in neutral position
                 1.0, // Stabilize mount's roll angle, enable roll stabilisation relative to Earth
                 1.0, // Stabilize mount's pitch/tilt angle, enable tilt/pitch stabilisation relative to Earth
                 1.0, // Stabilize mount pan/yaw angle, enable pan/yaw stabilisation relative to Earth
                12.0, // roll RC input channel, 0 for none, any other for the RC channel to be used to control roll movements
             17999.0, // Minimum roll angle, Minimum physical roll angular position of mount.
             17999.0, // Maximum roll angle, Maximum physical roll angular position of the mount
                12.0, // tilt (pitch) RC input channel, 0 for none, any other for the RC channel to be used to control tilt (pitch) movements
             17999.0, // Minimum tilt angle, Minimum physical tilt (pitch) angular position of mount.
             17999.0, // Maximum tilt angle, Maximum physical tilt (pitch) angular position of the mount
                12.0, // pan (yaw) RC input channel, 0 for none, any other for the RC channel to be used to control pan (yaw) movements
             17999.0, // Minimum pan angle, Minimum physical pan (yaw) angular position of mount.
             17999.0, // Maximum pan angle, Maximum physical pan (yaw) angular position of the mount
               100.0, // mount joystick speed, 0 for position control, small for low speeds, 100 for max speed. A good general value is 10 which gives a movement speed of 3 degrees per second.
                 0.2, // Roll stabilization lead time, Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate, compensating for servo delay. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.
                 0.2, // Pitch stabilization lead time, Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.
                 5.0, // Mount Type, Mount Type (None, Servo or MAVLink)
                 4.0, // Mount default operating mode, Mount default operating mode on startup and after control is returned from autopilot
              179.99, // Mount2 roll angle when in retracted position, Mount2 roll angle when in retracted position
              179.99, // Mount2 tilt/pitch angle when in retracted position, Mount2 tilt/pitch angle when in retracted position
              179.99, // Mount2 yaw/pan angle when in retracted position, Mount2 yaw/pan angle when in retracted position
              179.99, // Mount2 roll angle when in neutral position, Mount2 roll angle when in neutral position
              179.99, // Mount2 tilt/pitch angle when in neutral position, Mount2 tilt/pitch angle when in neutral position
              179.99, // Mount2 pan/yaw angle when in neutral position, Mount2 pan/yaw angle when in neutral position
                 1.0, // Stabilize Mount2's roll angle, enable roll stabilisation relative to Earth
                 1.0, // Stabilize Mount2's pitch/tilt angle, enable tilt/pitch stabilisation relative to Earth
                 1.0, // Stabilize mount2 pan/yaw angle, enable pan/yaw stabilisation relative to Earth
                12.0, // Mount2's roll RC input channel, 0 for none, any other for the RC channel to be used to control roll movements
             17999.0, // Mount2's minimum roll angle, Mount2's minimum physical roll angular position
             17999.0, // Mount2's maximum roll angle, Mount2's maximum physical roll angular position
                12.0, // Mount2's tilt (pitch) RC input channel, 0 for none, any other for the RC channel to be used to control tilt (pitch) movements
             17999.0, // Mount2's minimum tilt angle, Mount2's minimum physical tilt (pitch) angular position
             17999.0, // Mount2's maximum tilt angle, Mount2's maximum physical tilt (pitch) angular position
                12.0, // Mount2's pan (yaw) RC input channel, 0 for none, any other for the RC channel to be used to control pan (yaw) movements
             17999.0, // Mount2's minimum pan angle, Mount2's minimum physical pan (yaw) angular position
             17999.0, // Mount2's maximum pan angle, MOunt2's maximum physical pan (yaw) angular position
                 0.2, // Mount2's Roll stabilization lead time, Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate, compensating for servo delay. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.
                 0.2, // Mount2's Pitch stabilization lead time, Causes the servo angle output to lead the current angle of the vehicle by some amount of time based on current angular rate. Increase until the servo is responsive but doesn't overshoot. Does nothing with pan stabilization enabled.
                 5.0, // Mount2 Type, Mount Type (None, Servo or MAVLink)
                 3.0, // DataFlash Backend Storage type, 0 for None, 1 for File, 2 for dataflash mavlink, 3 for both file and dataflash
         3.40282e+38, // Maximum DataFlash File Backend buffer size (in kilobytes), The DataFlash_File backend uses a buffer to store data before writing to the block device.  Raising this value may reduce "gaps" in your SD card logging.  This buffer size may be reduced depending on available memory.  PixHawk requires at least 4 kilobytes.  Maximum value available here is 64 kilobytes.
                 1.0, // Enable logging while disarmed, If LOG_DISARMED is set to 1 then logging will be enabled while disarmed. This can make for very large logfiles but can help a lot when tracking down startup issues
                 1.0, // Enable logging of information needed for Replay, If LOG_REPLAY is set to 1 then the EKF2 state estimator will log detailed information needed for diagnosing problems with the Kalman filter. It is suggested that you also raise LOG_FILE_BUFSIZE to give more buffer space for logging and use a high quality microSD card to ensure no sensor data is lost
                 6.0, // Battery monitoring, Controls enabling monitoring of the battery's voltage and current
               100.0, // Battery Voltage sensing pin, Setting this to 0 ~ 13 will enable battery voltage sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 13. On the PX4 it should be set to 100. On the Pixhawk powered from the PM connector it should be set to 2.
               101.0, // Battery Current sensing pin, Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 12. On the PX4 it should be set to 101. On the Pixhawk powered from the PM connector it should be set to 3.
         3.40282e+38, // Voltage Multiplier, Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick on APM2 or Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX4 using the PX4IO power supply this should be set to 1.
         3.40282e+38, // Amps per volt, Number of amps that a 1V reading on the current sensor corresponds to. On the APM2 or Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.
         3.40282e+38, // AMP offset, Voltage offset at zero current on current sensor
         3.40282e+38, // Battery capacity, Capacity of the battery in mAh when full
         3.40282e+38, // Maximum allowed power (Watts), If battery wattage (voltage * current) exceeds this value then the system will reduce max throttle (THR_MAX, TKOFF_THR_MAX and THR_MIN for reverse thrust) to satisfy this limit. This helps limit high current to low C rated batteries regardless of battery voltage. The max throttle will slowly grow back to THR_MAX (or TKOFF_THR_MAX ) and THR_MIN if demanding the current max and under the watt max. Use 0 to disable.
                 6.0, // Battery monitoring, Controls enabling monitoring of the battery's voltage and current
               100.0, // Battery Voltage sensing pin, Setting this to 0 ~ 13 will enable battery voltage sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 13. On the PX4 it should be set to 100. On the Pixhawk powered from the PM connector it should be set to 2.
               101.0, // Battery Current sensing pin, Setting this to 0 ~ 13 will enable battery current sensing on pins A0 ~ A13. For the 3DR power brick on APM2.5 it should be set to 12. On the PX4 it should be set to 101. On the Pixhawk powered from the PM connector it should be set to 3.
         3.40282e+38, // Voltage Multiplier, Used to convert the voltage of the voltage sensing pin (BATT_VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick on APM2 or Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX4 using the PX4IO power supply this should be set to 1.
         3.40282e+38, // Amps per volt, Number of amps that a 1V reading on the current sensor corresponds to. On the APM2 or Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17.
         3.40282e+38, // AMP offset, Voltage offset at zero current on current sensor
         3.40282e+38, // Battery capacity, Capacity of the battery in mAh when full
         3.40282e+38, // Maximum allowed current, If battery wattage (voltage * current) exceeds this value then the system will reduce max throttle (THR_MAX, TKOFF_THR_MAX and THR_MIN for reverse thrust) to satisfy this limit. This helps limit high current to low C rated batteries regardless of battery voltage. The max throttle will slowly grow back to THR_MAX (or TKOFF_THR_MAX ) and THR_MIN if demanding the current max and under the watt max. Use 0 to disable.
                 7.0, // Auxiliary pin config, Control assigning of FMU pins to PWM output, timer capture and GPIO. All unassigned pins can be used for GPIO
                 2.0, // Serial 1 flow control, Enable flow control on serial 1 (telemetry 1) on Pixhawk. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup. Note that the PX4v1 does not have hardware flow control pins on this port, so you should leave this disabled.
                 2.0, // Serial 2 flow control, Enable flow control on serial 2 (telemetry 2) on Pixhawk and PX4. You must have the RTS and CTS pins connected to your radio. The standard DF13 6 pin connector for a 3DR radio does have those pins connected. If this is set to 2 then flow control will be auto-detected by checking for the output buffer filling on startup.
                 1.0, // Enable use of safety arming switch, This controls the default state of the safety switch at startup. When set to 1 the safety switch will start in the safe state (flashing) at boot. When set to zero the safety switch will start in the unsafe state (solid) at startup. Note that if a safety switch is fitted the user can still control the safety state after startup using the switch. The safety state can also be controlled in software using a MAVLink message.
                 7.0, //  SBUS output rate, This sets the SBUS output frame rate in Hz
             32768.0, // User-defined serial number, User-defined serial number of this vehicle, it can be any arbitrary number you want and has no effect on the autopilot
                 1.0, //  Enable use of UAVCAN devices, Enabling this option on a Pixhawk enables UAVCAN devices. Note that this uses about 25k of memory
                 1.0, // Channels to which ignore the safety switch state, A bitmask which controls what channels can move while the safety switch has not been pressed
                80.0, // Target IMU temperature, This sets the target IMU temperature for boards with controllable IMU heating units. A value of -1 disables heating.
                12.0, // Board type, This allows selection of a PX4 or VRBRAIN board type. If set to zero then the board type is auto-detected (PX4)
                 1.0, // Sprayer enable/disable, Allows you to enable (1) or disable (0) the sprayer
               100.0, // Pump speed, Desired pump speed when travelling 1m/s expressed as a percentage
              2000.0, // Spinner rotation speed, Spinner's rotation speed in PWM (a higher rate will disperse the spray over a wider area horizontally)
              1000.0, // Speed minimum, Speed minimum at which we will begin spraying
               100.0, // Pump speed minimum, Minimum pump speed expressed as a percentage
         3.40282e+38, // Absolute Pressure, calibrated ground pressure in Pascals
         3.40282e+38, // ground temperature, calibrated ground temperature in degrees Celsius
         3.40282e+38, // altitude offset, altitude offset in meters added to barometric altitude. This is used to allow for automatic adjustment of the base barometric altitude by a ground station equipped with a barometer. The value is added to the barometric altitude read by the aircraft. It is automatically reset to 0 when the barometer is calibrated on each reboot or when a preflight calibration is performed.
                 2.0, // Primary barometer, This selects which barometer will be the primary if multiple barometers are found
                11.0, // GPS type, GPS type
                11.0, // 2nd GPS type, GPS type of 2nd GPS
                 8.0, // Navigation filter setting, Navigation filter engine setting
                 1.0, // Automatic Switchover Setting, Automatic switchover to GPS reporting best lock
               100.0, // Minimum Lock Type Accepted for DGPS, Sets the minimum type of differential GPS corrections required before allowing to switch into DGPS mode.
                 2.0, // SBAS Mode, This sets the SBAS (satellite based augmentation system) mode if available on this GPS. If set to 2 then the SBAS mode is not changed in the GPS. Otherwise the GPS will be reconfigured to enable/disable SBAS. Disabling SBAS may be worthwhile in some parts of the world where an SBAS signal is available but the baseline is too long to be useful.
                90.0, // Minimum elevation, This sets the minimum elevation of satellites above the horizon for them to be used for navigation. Setting this to -100 leaves the minimum elevation set to the GPS modules default.
               127.0, // Destination for GPS_INJECT_DATA MAVLink packets, The GGS can send raw serial packets to inject data to multiple GPSes.
               65535, // Swift Binary Protocol Logging Mask, Masked with the SBP msg_type field to determine whether SBR1/SBR2 data is logged
                 5.0, // Raw data logging, Enable logging of RXM raw data from uBlox which includes carrier phase and pseudo range information. This allows for post processing of dataflash logs for more precise positioning. Note that this requires a raw capable uBlox such as the 6P or 6T.
                67.0, // GNSS system configuration, Bitmask for what GNSS system to use on the first GPS (all unchecked or zero to leave GPS as configured)
                 2.0, // Save GPS configuration, Determines whether the configuration for this GPS should be written to non-volatile memory on the GPS. Currently working for UBlox 6 series and above.
                67.0, // GNSS system configuration, Bitmask for what GNSS system to use on the second GPS (all unchecked or zero to leave GPS as configured)
                 1.0, // Automatic GPS configuration, Controls if the autopilot should automatically configure the GPS based on the parameters and default settings
                 3.0, // Scheduler debug level, Set to non-zero to enable scheduler debug messages. When set to show "Slips" the scheduler will display a message whenever a scheduled task is delayed due to too much CPU load. When set to ShowOverruns the scheduled will display a message whenever a task takes longer than the limit promised in the task table.
               400.0, // Scheduling main loop rate, This controls the rate of the main control loop in Hz. This should only be changed by developers. This only takes effect on restart
                 1.0, // Fence enable/disable, Allows you to enable (1) or disable (0) the fence functionality
                 7.0, // Fence Type, Enabled fence types held as bitmask
                 1.0, // Fence Action, What action should be taken when fence is breached
              1000.0, // Fence Maximum Altitude, Maximum altitude allowed before geofence triggers
             10000.0, // Circular Fence Radius, Circle fence radius which when breached will cause an RTL
                10.0, // Fence Margin, Distance that autopilot's should maintain from the fence to avoid a breach
                20.0, // Fence polygon point total, Number of polygon points saved in eeprom (do not update manually)
                 1.0, // Avoidance control enable/disable, Enabled/disable stopping at fence
         3.40282e+38, // Rally Total, Number of rally points currently loaded
         3.40282e+38, // Rally Limit, Maximum distance to rally point. If the closest rally point is more than this number of kilometers from the current position and the home location is closer than any of the rally points from the current position then do RTL to home rather than to the closest rally point. This prevents a leftover rally point from a different airfield being used accidentally. If this is set to 0 then the closest rally point is always used.
                 1.0, // Rally Include Home, Controls if Home is included as a Rally point (i.e. as a safe landing place) for RTL
               180.0, // Servo 1 Position, Angular location of swash servo #1
               180.0, // Servo 2 Position, Angular location of swash servo #2
               180.0, // Servo 3 Position, Angular location of swash servo #3
                 3.0, // Tail Type, Tail type selection.  Simpler yaw controller used if external gyro is selected
                 1.0, // Swash Type, Swash Type Setting - either 3-servo CCPM or H1 Mechanical Mixing
              1000.0, // External Gyro Gain, PWM sent to external gyro on ch7 when tail type is Servo w/ ExtGyro
                90.0, // Swashplate Phase Angle Compensation, Phase angle correction for rotor head.  If pitching the swash forward induces a roll, this can be correct the problem
                10.0, // Collective-Yaw Mixing, Feed-forward compensation to automatically add rudder input when collective pitch is increased. Can be positive or negative depending on mechanics.
                 1.0, // Flybar Mode Selector, Flybar present or not.  Affects attitude controller used during ACRO flight mode
              1000.0, // Direct Drive VarPitch Tail ESC speed, Direct Drive VarPitch Tail ESC speed.  Only used when TailType is DirectDrive VarPitch
              1000.0, // External Gyro Gain for ACRO, PWM sent to external gyro on ch7 when tail type is Servo w/ ExtGyro. A value of zero means to use H_GYR_GAIN
              2000.0, // RSC PWM output miniumum, This sets the PWM output on RSC channel for maximum rotor speed
              2000.0, // RSC PWM output maxiumum, This sets the PWM output on RSC channel for miniumum rotor speed
                 1.0, // RSC PWM reversal, This controls reversal of the RSC channel output
               500.0, // Matrix Yaw Min, Yaw control is given at least this pwm range
                 0.8, // Thrust Curve Expo, Motor thrust curve exponent (from 0 for linear to 1.0 for second order curve)
                 1.0, // Motor Spin maximum, Point at which the thrust saturates expressed as a number from 0 to 1 in the entire output range
                35.0, // Battery voltage compensation maximum voltage, Battery voltage compensation maximum voltage (voltage above this will have no additional scaling effect on thrust).  Recommend 4.4 * cell count, 0 = Disabled
                35.0, // Battery voltage compensation minimum voltage, Battery voltage compensation minimum voltage (voltage below this will have no additional scaling effect on thrust).  Recommend 3.5 * cell count, 0 = Disabled
               200.0, // Motor Current Max, Maximum current over which maximum throttle is limited (0 = Disabled)
                 2.0, // Output PWM type, This selects the output PWM type, allowing for normal PWM continuous output or OneShot125
              2000.0, // PWM output miniumum, This sets the min PWM output value that will ever be output to the motors, 0 = use input RC3_MIN
              2000.0, // PWM output maximum, This sets the max PWM value that will ever be output to the motors, 0 = use input RC3_MAX
                 0.3, // Motor Spin minimum, Point at which the thrust starts expressed as a number from 0 to 1 in the entire output range
                 0.2, // Motor Spin armed, Point at which the motors start to spin expressed as a number from 0 to 1 in the entire output range
                10.0, // Motor Current Max Time Constant, Time constant used to limit the maximum current
                 0.8, // Thrust Hover Value, Motor thrust needed to hover expressed as a number from 0 to 1
                 2.0, // Hover Value Learning, Enable/Disable automatic learning of hover throttle
                 8.0, // Roll channel, Roll channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Roll is normally on channel 1, but you can move it to any channel with this parameter.  Reboot is required for changes to take effect.
                 8.0, // Pitch channel, Pitch channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Pitch is normally on channel 2, but you can move it to any channel with this parameter.  Reboot is required for changes to take effect.
                 8.0, // Throttle channel, Throttle channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Throttle is normally on channel 3, but you can move it to any channel with this parameter. Warning APM 2.X: Changing the throttle channel could produce unexpected fail-safe results if connection between receiver and on-board PPM Encoder is lost. Disabling on-board PPM Encoder is recommended.  Reboot is required for changes to take effect.
                 8.0, // Yaw channel, Yaw channel number. This is useful when you have a RC transmitter that can't change the channel order easily. Yaw (also known as rudder) is normally on channel 4, but you can move it to any channel with this parameter.  Reboot is required for changes to take effect.
                 1.0, // Enable EKF1, This enables EKF1 to be disabled when using alternative algorithms. When disabling it, the alternate EKF2 estimator must be enabled by setting EK2_ENABLED = 1 and flight control algorithms must be set to use the alternative estimator by setting AHRS_EKF_TYPE = 2.
                 5.0, // GPS horizontal velocity measurement noise scaler, This is the scaler that is applied to the speed accuracy reported by the receiver to estimate the horizontal velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then a speed accuracy of 1 is assumed. Increasing it reduces the weighting on these measurements.
                 5.0, // GPS vertical velocity measurement noise scaler, This is the scaler that is applied to the speed accuracy reported by the receiver to estimate the vertical velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then a speed accuracy of 1 is assumed. Increasing it reduces the weighting on this measurement.
                10.0, // GPS horizontal position measurement noise (m), This is the RMS value of noise in the GPS horizontal position measurements. Increasing it reduces the weighting on these measurements.
                10.0, // Altitude measurement noise (m), This is the RMS value of noise in the altitude measurement. Increasing it reduces the weighting on this measurement.
                 0.5, // Magnetometer measurement noise (Gauss), This is the RMS value of noise in magnetometer measurements. Increasing it reduces the weighting on these measurements.
                 5.0, // Equivalent airspeed measurement noise (m/s), This is the RMS value of noise in equivalent airspeed measurements. Increasing it reduces the weighting on these measurements.
                 1.0, // Wind velocity process noise (m/s^2), This noise controls the growth of wind state error estimates. Increasing it makes wind estimation faster and noisier.
                 1.0, // Height rate to wind procss noise scaler, Increasing this parameter increases how rapidly the wind states adapt when changing altitude, but does make wind speed estimation noiser.
                0.05, // Rate gyro noise (rad/s), This noise controls the growth of estimated error due to gyro measurement errors excluding bias. Increasing it makes the flter trust the gyro measurements less and other measurements more.
                 1.0, // Accelerometer noise (m/s^2), This noise controls the growth of estimated error due to accelerometer measurement errors excluding bias. Increasing it makes the flter trust the accelerometer measurements less and other measurements more.
               1e-05, // Rate gyro bias process noise (rad/s), This noise controls the growth of gyro bias state error estimates. Increasing it makes rate gyro bias estimation faster and noisier.
               0.001, // Accelerometer bias process noise (m/s^2), This noise controls the growth of the vertical acelerometer bias state error estimate. Increasing it makes accelerometer bias estimation faster and noisier.
                0.01, // Earth magnetic field process noise (gauss/s), This noise controls the growth of earth magnetic field state error estimates. Increasing it makes earth magnetic field bias estimation faster and noisier.
                0.01, // Body magnetic field process noise (gauss/s), This noise controls the growth of body magnetic field state error estimates. Increasing it makes compass offset estimation faster and noisier.
               500.0, // GPS velocity measurement delay (msec), This is the number of msec that the GPS velocity measurements lag behind the inertial measurements.
               500.0, // GPS position measurement delay (msec), This is the number of msec that the GPS position measurements lag behind the inertial measurements.
                 3.0, // GPS mode control, This parameter controls use of GPS measurements : 0 = use 3D velocity & 2D position, 1 = use 2D velocity and 2D position, 2 = use 2D position, 3 = use no GPS (optical flow will be used if available)
               100.0, // GPS velocity measurement gate size, This parameter sets the number of standard deviations applied to the GPS velocity measurement innovation consistency check. Decreasing it makes it more likely that good measurements willbe rejected. Increasing it makes it more likely that bad measurements will be accepted.
               100.0, // GPS position measurement gate size, This parameter sets the number of standard deviations applied to the GPS position measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
               100.0, // Height measurement gate size, This parameter sets the number of standard deviations applied to the height measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
               100.0, // Magnetometer measurement gate size, This parameter sets the number of standard deviations applied to the magnetometer measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
               100.0, // Airspeed measurement gate size, This parameter sets the number of standard deviations applied to the airspeed measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
                 3.0, // Magnetometer calibration mode, EKF_MAG_CAL = 0 enables calibration based on flying speed and altitude and is the default setting for Plane users. EKF_MAG_CAL = 1 enables calibration based on manoeuvre level and is the default setting for Copter and Rover users. EKF_MAG_CAL = 2 prevents magnetometer calibration regardless of flight condition and is recommended if in-flight magnetometer calibration is unreliable.
               500.0, // GPS glitch accel gate size (cm/s^2), This parameter controls the maximum amount of difference in horizontal acceleration between the value predicted by the filter and the value measured by the GPS before the GPS position data is rejected. If this value is set too low, then valid GPS data will be regularly discarded, and the position accuracy will degrade. If this parameter is set too high, then large GPS glitches will cause large rapid changes in position.
                50.0, // GPS glitch radius gate size (m), This parameter controls the maximum amount of difference in horizontal position (in m) between the value predicted by the filter and the value measured by the GPS before the long term glitch protection logic is activated and the filter states are reset to the new GPS position. Position steps smaller than this value will be temporarily ignored, but will then be accepted and the filter will move to the new position. Position steps larger than this value will be ignored initially, but the filter will then apply an offset to the GPS position measurement.
                50.0, // Terrain Gradient % RMS, This parameter sets the RMS terrain gradient percentage assumed by the terrain height estimation. Terrain height can be estimated using optical flow and/or range finder sensor data if fitted. Smaller values cause the terrain height estimate to be slower to respond to changes in measurement. Larger values cause the terrain height estimate to be faster to respond, but also more noisy. Generally this value can be reduced if operating over very flat terrain and increased if operating over uneven terrain.
                 1.0, // Optical flow measurement noise (rad/s), This is the RMS value of noise and errors in optical flow measurements. Increasing it reduces the weighting on these measurements.
               100.0, // Optical Flow measurement gate size, This parameter sets the number of standard deviations applied to the optical flow innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
               500.0, // Optical Flow measurement delay (msec), This is the number of msec that the optical flow measurements lag behind the inertial measurements. It is the time from the end of the optical flow averaging period and does not include the time delay due to the 100msec of averaging within the flow sensor.
               100.0, // Range finder measurement gate size, This parameter sets the number of standard deviations applied to the range finder innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
                 4.0, // Maximum valid optical flow rate, This parameter sets the magnitude maximum optical flow rate in rad/sec that will be accepted by the filter
                 1.0, // Fallback strictness, This parameter controls the conditions necessary to trigger a fallback to DCM and INAV. A value of 1 will cause fallbacks to occur on loss of GPS and other conditions. A value of 0 will trust the EKF more.
                 1.0, // Primary height source, This parameter controls which height sensor is used by the EKF during optical flow navigation (when EKF_GPS_TYPE = 3). A value of will 0 cause it to always use baro altitude. A value of 1 will cause it to use range finder if available.
         3.40282e+38, // GPS preflight check, 1 byte bitmap of GPS preflight checks to perform. Set to 0 to bypass all checks. Set to 255 perform all checks. Set to 3 to check just the number of satellites and HDoP. Set to 31 for the most rigorous checks that will still allow checks to pass when the copter is moving, eg launch from a boat.
                 1.0, // Enable EKF2, This enables EKF2. Enabling EKF2 only makes the maths run, it does not mean it will be used for flight control. To use it for flight control set AHRS_EKF_TYPE=2. A reboot or restart will need to be performed after changing the value of EK2_ENABLE for it to take effect.
                 3.0, // GPS mode control, This controls use of GPS measurements : 0 = use 3D velocity & 2D position, 1 = use 2D velocity and 2D position, 2 = use 2D position, 3 = use no GPS (optical flow will be used if available)
                 5.0, // GPS horizontal velocity measurement noise (m/s), This sets a lower limit on the speed accuracy reported by the GPS receiver that is used to set horizontal velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then the parameter value will be used. Increasing it reduces the weighting of the GPS horizontal velocity measurements.
                 5.0, // GPS vertical velocity measurement noise (m/s), This sets a lower limit on the speed accuracy reported by the GPS receiver that is used to set vertical velocity observation noise. If the model of receiver used does not provide a speed accurcy estimate, then the parameter value will be used. Increasing it reduces the weighting of the GPS vertical velocity measurements.
              1000.0, // GPS velocity innovation gate size, This sets the percentage number of standard deviations applied to the GPS velocity measurement innovation consistency check. Decreasing it makes it more likely that good measurements willbe rejected. Increasing it makes it more likely that bad measurements will be accepted.
                10.0, // GPS horizontal position measurement noise (m), This sets the GPS horizontal position observation noise. Increasing it reduces the weighting of GPS horizontal position measurements.
              1000.0, // GPS position measurement gate size, This sets the percentage number of standard deviations applied to the GPS position measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
               100.0, // GPS glitch radius gate size (m), This controls the maximum radial uncertainty in position between the value predicted by the filter and the value measured by the GPS before the filter position and velocity states are reset to the GPS. Making this value larger allows the filter to ignore larger GPS glitches but also means that non-GPS errors such as IMU and compass can create a larger error in position before the filter is forced back to the GPS position.
               250.0, // GPS measurement delay (msec), This is the number of msec that the GPS measurements lag behind the inertial measurements.
                 2.0, // Primary height source, This parameter controls the primary height sensor used by the EKF. If the selected option cannot be used, it will default to Baro as the primary height source. Setting 0 will use the baro altitude at all times. Setting 1 uses the range finder and is only available in combination with optical flow navigation (EK2_GPS_TYPE = 3). Setting 2 uses GPS. NOTE - the EK2_RNG_USE_HGT parameter can be used to switch to range-finder when close to the ground.
                10.0, // Altitude measurement noise (m), This is the RMS value of noise in the altitude measurement. Increasing it reduces the weighting of the baro measurement and will make the filter respond more slowly to baro measurement errors, but will make it more sensitive to GPS and accelerometer errors.
              1000.0, // Height measurement gate size, This sets the percentage number of standard deviations applied to the height measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
               250.0, // Height measurement delay (msec), This is the number of msec that the Height measurements lag behind the inertial measurements.
                 0.5, // Magnetometer measurement noise (Gauss), This is the RMS value of noise in magnetometer measurements. Increasing it reduces the weighting on these measurements.
                 4.0, // Magnetometer calibration mode, EKF_MAG_CAL = 0 enables calibration when airborne and is the default setting for Plane users. EKF_MAG_CAL = 1 enables calibration when manoeuvreing. EKF_MAG_CAL = 2 prevents magnetometer calibration regardless of flight condition, is recommended if the external magnetic field is varying and is the default for rovers. EKF_MAG_CAL = 3 enables calibration when the first in-air field and yaw reset has completed and is the default for copters. EKF_MAG_CAL = 4 enables calibration all the time. This determines when the filter will use the 3-axis magnetometer fusion model that estimates both earth and body fixed magnetic field states. This model is only suitable for use when the external magnetic field environment is stable.
              1000.0, // Magnetometer measurement gate size, This sets the percentage number of standard deviations applied to the magnetometer measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
                 5.0, // Equivalent airspeed measurement noise (m/s), This is the RMS value of noise in equivalent airspeed measurements used by planes. Increasing it reduces the weighting of airspeed measurements and will make wind speed estimates less noisy and slower to converge. Increasing also increases navigation errors when dead-reckoning without GPS measurements.
              1000.0, // Airspeed measurement gate size, This sets the percentage number of standard deviations applied to the airspeed measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
                10.0, // Range finder measurement noise (m), This is the RMS value of noise in the range finder measurement. Increasing it reduces the weighting on this measurement.
              1000.0, // Range finder measurement gate size, This sets the percentage number of standard deviations applied to the range finder innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
                 4.0, // Maximum valid optical flow rate, This sets the magnitude maximum optical flow rate in rad/sec that will be accepted by the filter
                 1.0, // Optical flow measurement noise (rad/s), This is the RMS value of noise and errors in optical flow measurements. Increasing it reduces the weighting on these measurements.
              1000.0, // Optical Flow measurement gate size, This sets the percentage number of standard deviations applied to the optical flow innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
               250.0, // Optical Flow measurement delay (msec), This is the number of msec that the optical flow measurements lag behind the inertial measurements. It is the time from the end of the optical flow averaging period and does not include the time delay due to the 100msec of averaging within the flow sensor.
                 0.1, // Rate gyro noise (rad/s), This control disturbance noise controls the growth of estimated error due to gyro measurement errors excluding bias. Increasing it makes the flter trust the gyro measurements less and other measurements more.
                 1.0, // Accelerometer noise (m/s^2), This control disturbance noise controls the growth of estimated error due to accelerometer measurement errors excluding bias. Increasing it makes the flter trust the accelerometer measurements less and other measurements more.
               0.001, // Rate gyro bias stability (rad/s/s), This state  process noise controls growth of the gyro delta angle bias state error estimate. Increasing it makes rate gyro bias estimation faster and noisier.
               0.001, // Rate gyro scale factor stability (1/s), This noise controls the rate of gyro scale factor learning. Increasing it makes rate gyro scale factor estimation faster and noisier.
               0.001, // Accelerometer bias stability (m/s^3), This noise controls the growth of the vertical accelerometer delta velocity bias state error estimate. Increasing it makes accelerometer bias estimation faster and noisier.
                 1.0, // Wind velocity process noise (m/s^2), This state process noise controls the growth of wind state error estimates. Increasing it makes wind estimation faster and noisier.
                 1.0, // Height rate to wind procss noise scaler, This controls how much the process noise on the wind states is increased when gaining or losing altitude to take into account changes in wind speed and direction with altitude. Increasing this parameter increases how rapidly the wind states adapt when changing altitude, but does make wind velocity estimation noiser.
         3.40282e+38, // GPS preflight check, This is a 1 byte bitmap controlling which GPS preflight checks are performed. Set to 0 to bypass all checks. Set to 255 perform all checks. Set to 3 to check just the number of satellites and HDoP. Set to 31 for the most rigorous checks that will still allow checks to pass when the copter is moving, eg launch from a boat.
               127.0, // Bitmask of active IMUs, 1 byte bitmap of IMUs to use in EKF2. A separate instance of EKF2 will be started for each IMU selected. Set to 1 to use the first IMU only (default), set to 2 to use the second IMU only, set to 3 to use the first and second IMU. Additional IMU's can be used up to a maximum of 6 if memory and processing resources permit. There may be insufficient memory and processing resources to run multiple instances. If this occurs EKF2 will fail to start.
               200.0, // GPS accuracy check scaler (%), This scales the thresholds that are used to check GPS accuracy before it is used by the EKF. A value of 100 is the default. Values greater than 100 increase and values less than 100 reduce the maximum GPS error the EKF will accept. A value of 200 will double the allowable GPS error.
                50.0, // Non-GPS operation position uncertainty (m), This sets the amount of position variation that the EKF allows for when operating without external measurements (eg GPS or optical flow). Increasing this parameter makes the EKF attitude estimate less sensitive to vehicle manoeuvres but more sensitive to IMU errors.
                 7.0, // EKF sensor logging IMU mask, This sets the IMU mask of sensors to do full logging for
                 1.0, // Yaw measurement noise (rad), This is the RMS value of noise in yaw measurements from the magnetometer. Increasing it reduces the weighting on these measurements.
              1000.0, // Yaw measurement gate size, This sets the percentage number of standard deviations applied to the magnetometer yaw measurement innovation consistency check. Decreasing it makes it more likely that good measurements will be rejected. Increasing it makes it more likely that bad measurements will be accepted.
                50.0, // Output complementary filter time constant (centi-sec), Sets the time constant of the output complementary filter/predictor in centi-seconds.
                0.01, // Earth magnetic field process noise (gauss/s), This state process noise controls the growth of earth magnetic field state error estimates. Increasing it makes earth magnetic field estimation faster and noisier.
                0.01, // Body magnetic field process noise (gauss/s), This state process noise controls the growth of body magnetic field state error estimates. Increasing it makes magnetometer bias error estimation faster and noisier.
                70.0, // Range finder switch height percentage, The range finder will be used as the primary height source when below a specified percentage of the sensor maximum as set by the RNGFND_MAX_CM parameter. Set to -1 to prevent range finder use.
             32766.0, // Total mission commands, The number of mission mission items that has been loaded by the ground station. Do not change this manually.
                 1.0, // Mission Restart when entering Auto mode, Controls mission starting point when entering Auto mode (either restart from beginning of mission or resume from last command run)
                 2.0, // RSSI Type, Radio Receiver RSSI type. If your radio receiver supports RSSI of some kind, set it here, then set its associated RSSI_XXXXX parameters, if any.
               103.0, // Receiver RSSI analog sensing pin, This selects an analog pin where the receiver RSSI voltage will be read.
                 5.0, // Receiver RSSI voltage low, This is the voltage value that the radio receiver will put on the RSSI_ANA_PIN when the signal strength is the weakest. Since some radio receivers put out inverted values from what you might otherwise expect, this isn't necessarily a lower value than RSSI_PIN_HIGH. 
                 5.0, // Receiver RSSI voltage high, This is the voltage value that the radio receiver will put on the RSSI_ANA_PIN when the signal strength is the strongest. Since some radio receivers put out inverted values from what you might otherwise expect, this isn't necessarily a higher value than RSSI_PIN_LOW. 
         3.40282e+38, // Receiver RSSI channel number, The channel number where RSSI will be output by the radio receiver (5 and above).
              2000.0, // Receiver RSSI PWM low value, This is the PWM value that the radio receiver will put on the RSSI_CHANNEL when the signal strength is the weakest. Since some radio receivers put out inverted values from what you might otherwise expect, this isn't necessarily a lower value than RSSI_CHAN_HIGH. 
              2000.0, // Receiver RSSI PWM high value, This is the PWM value that the radio receiver will put on the RSSI_CHANNEL when the signal strength is the strongest. Since some radio receivers put out inverted values from what you might otherwise expect, this isn't necessarily a higher value than RSSI_CHAN_LOW. 
                10.0, // Rangefinder type, What type of rangefinder device that is connected
                64.0, // Rangefinder pin, Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
         3.40282e+38, // Rangefinder scaling, Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
         3.40282e+38, // rangefinder offset, Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM and I2C Lidars
                 2.0, // Rangefinder function, Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
         3.40282e+38, // Rangefinder minimum distance, Minimum distance in centimeters that rangefinder can reliably read
         3.40282e+38, // Rangefinder maximum distance, Maximum distance in centimeters that rangefinder can reliably read
               116.0, // Rangefinder stop pin, Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
         3.40282e+38, // Rangefinder settle time, The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
                 1.0, // Ratiometric, This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
             32767.0, // Powersave range, This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode (if available). A value of zero means power saving is not enabled
               127.0, // Distance (in cm) from the range finder to the ground, This parameter sets the expected range measurement(in cm) that the range finder should return when the vehicle is on the ground.
               127.0, // Bus address of sensor, This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
                10.0, // Second Rangefinder type, What type of rangefinder device that is connected
                64.0, // Rangefinder pin, Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
         3.40282e+38, // Rangefinder scaling, Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
         3.40282e+38, // rangefinder offset, Offset in volts for zero distance
                 2.0, // Rangefinder function, Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
         3.40282e+38, // Rangefinder minimum distance, Minimum distance in centimeters that rangefinder can reliably read
         3.40282e+38, // Rangefinder maximum distance, Maximum distance in centimeters that rangefinder can reliably read
               116.0, // Rangefinder stop pin, Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
         3.40282e+38, // Sonar settle time, The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
                 1.0, // Ratiometric, This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
               127.0, // Distance (in cm) from the second range finder to the ground, This parameter sets the expected range measurement(in cm) that the second range finder should return when the vehicle is on the ground.
               127.0, // Bus address of second rangefinder, This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
                10.0, // Third Rangefinder type, What type of rangefinder device that is connected
                64.0, // Rangefinder pin, Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
         3.40282e+38, // Rangefinder scaling, Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
         3.40282e+38, // rangefinder offset, Offset in volts for zero distance
                 2.0, // Rangefinder function, Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
         3.40282e+38, // Rangefinder minimum distance, Minimum distance in centimeters that rangefinder can reliably read
         3.40282e+38, // Rangefinder maximum distance, Maximum distance in centimeters that rangefinder can reliably read
               116.0, // Rangefinder stop pin, Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
         3.40282e+38, // Sonar settle time, The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
                 1.0, // Ratiometric, This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
               127.0, // Distance (in cm) from the third range finder to the ground, This parameter sets the expected range measurement(in cm) that the third range finder should return when the vehicle is on the ground.
               127.0, // Bus address of third rangefinder, This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
                10.0, // Fourth Rangefinder type, What type of rangefinder device that is connected
                64.0, // Rangefinder pin, Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
         3.40282e+38, // Rangefinder scaling, Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
         3.40282e+38, // rangefinder offset, Offset in volts for zero distance
                 2.0, // Rangefinder function, Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
         3.40282e+38, // Rangefinder minimum distance, Minimum distance in centimeters that rangefinder can reliably read
         3.40282e+38, // Rangefinder maximum distance, Maximum distance in centimeters that rangefinder can reliably read
               116.0, // Rangefinder stop pin, Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
         3.40282e+38, // Sonar settle time, The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
                 1.0, // Ratiometric, This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
               127.0, // Distance (in cm) from the fourth range finder to the ground, This parameter sets the expected range measurement(in cm) that the fourth range finder should return when the vehicle is on the ground.
               127.0, // Bus address of fourth rangefinder, This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
                 1.0, // Terrain data enable, enable terrain data. This enables the vehicle storing a database of terrain data on the SD card. The terrain data is requested from the ground station as needed, and stored for later use on the SD card. To be useful the ground station must support TERRAIN_REQUEST messages and have access to a terrain database, such as the SRTM database.
         3.40282e+38, // Terrain grid spacing, Distance between terrain grid points in meters. This controls the horizontal resolution of the terrain data that is stored on te SD card and requested from the ground station. If your GCS is using the worldwide SRTM database then a resolution of 100 meters is appropriate. Some parts of the world may have higher resolution data available, such as 30 meter data available in the SRTM database in the USA. The grid spacing also controls how much data is kept in memory during flight. A larger grid spacing will allow for a larger amount of data in memory. A grid spacing of 100 meters results in the vehicle keeping 12 grid squares in memory with each grid square having a size of 2.7 kilometers by 3.2 kilometers. Any additional grid squares are stored on the SD once they are fetched from the GCS and will be demand loaded as needed.
                 1.0, // Optical flow enable/disable, Setting this to Enabled(1) will enable optical flow. Setting this to Disabled(0) will disable optical flow
               200.0, // X axis optical flow scale factor correction, This sets the parts per thousand scale factor correction applied to the flow sensor X axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the X axis optical flow reading by 0.1%. Negative values reduce the scale factor.
               200.0, // Y axis optical flow scale factor correction, This sets the parts per thousand scale factor correction applied to the flow sensor Y axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the Y axis optical flow reading by 0.1%. Negative values reduce the scale factor.
             18000.0, // Flow sensor yaw alignment, Specifies the number of centi-degrees that the flow sensor is yawed relative to the vehicle. A sensor with its X-axis pointing to the right of the vehicle X axis has a positive yaw angle.
                 2.0, // Precision Land enabled/disabled and behaviour, Precision Land enabled/disabled and behaviour
                 2.0, // Precision Land Type, Precision Land Type
                 1.0, // RPM type, What type of RPM sensor is connected
         3.40282e+38, // RPM scaling, Scaling factor between sensor reading and RPM.
         3.40282e+38, // Maximum RPM, Maximum RPM to report
         3.40282e+38, // Minimum RPM, Minimum RPM to report
         3.40282e+38, // Minimum Quality, Minimum data quality to be used
                 1.0, // Second RPM type, What type of RPM sensor is connected
         3.40282e+38, // RPM scaling, Scaling factor between sensor reading and RPM.
                 1.0, // Enable ADSB, Enable ADS-B
               100.0, // ADSB vehicle list size, ADSB list size of nearest vehicles. Longer lists take longer to refresh with lower SRx_ADSB values.
            100000.0, // ADSB vehicle list radius filter, ADSB vehicle list radius filter. Vehicles detected outside this radius will be completely ignored. They will not show up in the SRx_ADSB stream to the GCS and will not be considered in any avoidance calculations.
          16777215.0, // ICAO_ID vehicle identifaction number, ICAO_ID unique vehicle identifaction number of this aircraft. This is a integer limited to 24bits. If set to 0 then one will be randomly generated. If set to -1 then static information is not sent, transceiver is assumed pre-programmed.
                19.0, // Emitter type, ADSB classification for the type of vehicle emitting the transponder signal. Default value is 14 (UAV).
                15.0, // Aircraft length and width, Aircraft length and width dimension options in Length and Width in meters. In most cases, use a value of 1 for smallest size.
                 7.0, // GPS antenna lateral offset, GPS antenna lateral offset. This describes the physical location offest from center of the GPS antenna on the aircraft.
                 1.0, // GPS antenna longitudinal offset, GPS antenna longitudinal offset. This is usually set to 1, Applied By Sensor
                 3.0, // Transceiver RF selection, Transceiver RF selection for Rx enable and/or Tx enable.
                 1.0, // Enable Avoidance using ADSB, Enable Avoidance using ADSB
                 1.0, // Recovery behaviour after a fail event, Determines what the aircraft will do after a fail event is resolved
         3.40282e+38, // Maximum number of obstacles to track, Maximum number of obstacles to track
         3.40282e+38, // Time Horizon Warn, Aircraft velocity vectors are multiplied by this time to determine closest approach.  If this results in an approach closer than W_DIST_XY or W_DIST_Z then W_ACTION is undertaken (assuming F_ACTION is not undertaken)
         3.40282e+38, // Time Horizon Fail, Aircraft velocity vectors are multiplied by this time to determine closest approach.  If this results in an approach closer than F_DIST_XY or F_DIST_Z then F_ACTION is undertaken
         3.40282e+38, // Distance Warn XY, Closest allowed projected distance before W_ACTION is undertaken
         3.40282e+38, // Distance Fail XY, Closest allowed projected distance before F_ACTION is undertaken
         3.40282e+38, // Distance Warn Z, Closest allowed projected distance before BEHAVIOUR_W is undertaken
         3.40282e+38, // Distance Fail Z, Closest allowed projected distance before BEHAVIOUR_F is undertaken
                 3.0, // LED Brightness, Select the RGB LED brightness level. When USB is connected brightness will never be higher than low regardless of the setting.
                 1.0, // Buzzer enable, Enable or disable the buzzer. Only for Linux and PX4 based boards.
                 1.0, // Setup for MAVLink LED override, This sets up the board RGB LED for override by MAVLink. Normal notify LED control is disabled
                 1.0, // Enable button reporting, This enables the button checking module. When this is disabled the parameters for setting button inputs are not visible
               116.0, // First button Pin, Digital pin number for first button input. 
               116.0, // Second button Pin, Digital pin number for second button input. 
               116.0, // Third button Pin, Digital pin number for third button input. 
               116.0, // Fourth button Pin, Digital pin number for fourth button input. 
              3600.0, // Report send time, The duration in seconds that a BUTTON_CHANGE report is repeatedly sent to the GCS regarding a button changing state. Note that the BUTTON_CHANGE message is MAVLink2 only.
};
#endif