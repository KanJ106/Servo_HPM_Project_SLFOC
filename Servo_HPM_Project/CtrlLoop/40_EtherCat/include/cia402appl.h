#ifndef _CIA402_H_
#define _CIA402_H_

#include "objdef.h"
#include "ecat_def.h"//
#include "ecatslv.h"
#include "ecatappl.h"
#include "coeappl.h"
#include "ecat_escinit.h"
#include "r_cg_Project.h"//

#define MAX_AXES    1

#if SERVOTYPE != SERVO_CANOPEN


//#define CIA402_OBJECTS_SIZE           (SIZEOF(CiA402Objects)/sizeof(UINT16))
//#define E0x6000_GROUP_SIZE		      (168)

/*---------------------------------------------
-    ControlWord Commands Mask (IEC61800_184e)
-----------------------------------------------*/
#define CONTROLWORD_COMMAND_SHUTDOWN_MASK                    0x0087 /**< \brief Shutdown command mask*/
#define CONTROLWORD_COMMAND_SWITCHON_MASK                    0x00C7 /**< \brief Switch on command mask*/
#define CONTROLWORD_COMMAND_SWITCHON_ENABLEOPERATION_MASK    0x008F /**< \brief Switch on & Enable command mask*/
#define CONTROLWORD_COMMAND_DISABLEVOLTAGE_MASK              0x0082 /**< \brief Disable voltage command mask*/
#define CONTROLWORD_COMMAND_QUICKSTOP_MASK                   0x0086 /**< \brief Quickstop command mask*/
#define CONTROLWORD_COMMAND_DISABLEOPERATION_MASK            0x008F /**< \brief Disable operation command mask*/
#define CONTROLWORD_COMMAND_ENABLEOPERATION_MASK             0x008F /**< \brief Enable operation command mask*/
#define CONTROLWORD_COMMAND_FAULTRESET_MASK                  0x0080 /**< \brief Fault reset command mask*/


/*---------------------------------------------
-    ControlWord Commands (IEC61800_184e)
-----------------------------------------------*/
#define CONTROLWORD_COMMAND_SHUTDOWN                         0x0006 /**< \brief Shutdown command*/
#define CONTROLWORD_COMMAND_SWITCHON                         0x0007 /**< \brief Switch on command*/
#define CONTROLWORD_COMMAND_SWITCHON_ENABLEOPERATION         0x000F /**< \brief Switch on & Enable command*/
#define CONTROLWORD_COMMAND_DISABLEVOLTAGE                   0x0000 /**< \brief Disable voltage command*/
#define CONTROLWORD_COMMAND_QUICKSTOP                        0x0002 /**< \brief Quickstop command*/
#define CONTROLWORD_COMMAND_DISABLEOPERATION                 0x0007 /**< \brief Disable operation command*/
#define CONTROLWORD_COMMAND_ENABLEOPERATION                  0x000F /**< \brief Enable operation command*/
#define CONTROLWORD_COMMAND_FAULTRESET                       0x0080 /**< \brief Fault reset command*/


/*---------------------------------------------
-    StatusWord Masks and Flags
-----------------------------------------------*/
#define STATUSWORD_STATE_MASK                                0x006F /**< \brief State mask*/
#define STATUSWORD_VOLTAGE_ENABLED                           0x0010 /**< \brief Indicate high voltage enabled*/
#define STATUSWORD_WARNING                                   0x0080 /**< \brief Warning active*/
#define STATUSWORD_MANUFACTORSPECIFIC                        0x0100 /**< \brief Manufacturer specific*/
#define STATUSWORD_INTERNAL_LIMIT                            0x0800 /**< \brief Internal limit*/
#define STATUSWORD_REMOTE                                    0x0200 /**< \brief Set if the control word is processed*/
#define STATUSWORD_TARGET_REACHED                            0x0400 /**< \brief Target reached*/
#define STATUSWORD_INTERNALLIMITACTIVE                       0x0800 /**< \brief Internal limit active*/
#define STATUSWORD_DRIVE_FOLLOWS_COMMAND                     0x1000 /**< \brief Drive follows command (used in cyclic synchronous modes)*/


/*---------------------------------------------
-    StatusWord
-----------------------------------------------*/
#define STATUSWORD_STATE_NOTREADYTOSWITCHON                  0x0000 /**< \brief Not ready to switch on*/
#define STATUSWORD_STATE_SWITCHEDONDISABLED                  0x0040 /**< \brief Switched on but disabled*/
#define STATUSWORD_STATE_READYTOSWITCHON                     0x0021 /**< \brief Ready to switch on*/
#define STATUSWORD_STATE_SWITCHEDON                          0x0023 /**< \brief Switched on*/
#define STATUSWORD_STATE_OPERATIONENABLED                    0x0027 /**< \brief Operation enabled*/
#define STATUSWORD_STATE_QUICKSTOPACTIVE                     0x0007 /**< \brief Quickstop active*/
#define STATUSWORD_STATE_FAULTREACTIONACTIVE                 0x000F /**< \brief Fault reaction active*/
#define STATUSWORD_STATE_FAULT                               0x0008 /**< \brief Fault state*/


/*---------------------------------------------
-    CiA402 State machine
-----------------------------------------------*/
#define STATE_NOT_READY_TO_SWITCH_ON        0x0001 /**< \brief Not ready to switch on (optional)*/
#define STATE_SWITCH_ON_DISABLED            0x0002 /**< \brief Switch on but disabled (optional)*/
#define STATE_READY_TO_SWITCH_ON            0x0004 /**< \brief Ready to switch on (mandatory)*/
#define STATE_SWITCHED_ON                   0x0008 /**< \brief Switch on (mandatory)*/
#define STATE_OPERATION_ENABLED             0x0010 /**< \brief Operation enabled (mandatory)*/
#define STATE_QUICK_STOP_ACTIVE             0x0020 /**< \brief Quick stop active (optional)*/
#define STATE_FAULT_REACTION_ACTIVE         0x0040 /**< \brief Fault reaction active (mandatory)*/
#define STATE_FAULT                         0x0080 /**< \brief Fault state (mandatory)*/


/*---------------------------------------------
-    CiA402 Modes of Operation (object 0x6060) (IEC61800_184e)
-----------------------------------------------*/
// -128 to -1 Manufacturer-specific operation modes
#define MIT_MODE                  (-6)
#define TORQUE_CLOSED_LOOP        (-1)
#define NO_MODE                     0 /**< \brief No mode*/
#define PROFILE_POSITION_MODE       1 /**< \brief Position Profile mode*/
#define VELOCITY_MODE               2 /**< \brief Velocity mode*/
#define PROFILE_VELOCITY_MOCE       3 /**< \brief Velocity Profile mode*/
#define PROFILE_TORQUE_MODE         4 /**< \brief Torque Profile mode*/
//5 reserved                          
#define HOMING_MODE                 6 /**< \brief Homing mode*/
#define INTERPOLATION_POSITION_MODE 7 /**< \brief Interpolation Position mode*/
#define CYCLIC_SYNC_POSITION_MODE   8 /**< \brief Cyclic Synchronous Position mode*/
#define CYCLIC_SYNC_VELOCITY_MODE   9 /**< \brief Cyclic Synchronous Velocity mode*/
#define CYCLIC_SYNC_TORQUE_MODE     10/**< \brief Cyclic Synchronous Torque mode*/
//+11 to +127 reserved

//TC200-N Supported Mode (object 0x6502)

#define SUPPORT_PP   0x001
#define SUPPORT_PV   0X004
#define SUPPORT_TQ   0x008
#define SUPPORT_HM   0x020
#define SUPPORT_IP   0x040
#define SUPPORT_CSP  0x080
#define SUPPORT_CSV  0x100
#define SUPPORT_CST  0x200


/***************************************
 CiA402 Error Codes (object 0x603F) (IEC61800_184e)
 ***************************************/

//#define ERROR_SHORT_CIRCUIT_EARTH_LEAKAGE_INPUT             0x2110 /**< \brief Short circuit/earth leakage (input)*/
//#define ERROR_EARTH_LEAKAGE_INPUT                           0x2120 /**< \brief Earth leakage (input)*/
//#define ERROR_EARTH_LEAKAGE_PHASE_L1                        0x2121 /**< \brief Earth leakage phase L1*/
//#define ERROR_EARTH_LEAKAGE_PHASE_L2                        0x2122 /**< \brief Earth leakage phase L2*/
//#define ERROR_EARTH_LEAKAGE_PHASE_L3                        0x2123 /**< \brief Earth leakage phase L3*/
//#define ERROR_SHORT_CIRCUIT_INPUT                           0x2130 /**< \brief Short circuit (input)*/
//#define ERROR_SHORT_CIRCUIT_PHASES_L1_L2                    0x2131 /**< \brief Short circuit phases L1-L2*/
//#define ERROR_SHORT_CIRCUIT_PHASES_L2_L3                    0x2132 /**< \brief Short circuit phases L2-L3*/
//#define ERROR_SHORT_CIRCUIT_PHASES_L3_L1                    0x2133 /**< \brief Short circuit phases L3-L1*/
//#define ERROR_INTERNAL_CURRENT_NO1                          0x2211 /**< \brief Internal current no 1*/
//#define ERROR_INTERNAL_CURRENT_NO2                          0x2212 /**< \brief Internal current no 2*/
//#define ERROR_OVER_CURRENT_IN_RAMP_FUNCTION                 0x2213 /**< \brief Over-current in ramp function*/
//#define ERROR_OVER_CURRENT_IN_THE_SEQUENCE                  0x2214 /**< \brief Over-current in the sequence*/
//#define ERROR_CONTINUOUS_OVER_CURRENT_DEVICE_INTERNAL       0x2220 /**< \brief Continuous over current (device internal)*/
//#define ERROR_CONTINUOUS_OVER_CURRENT_DEVICE_INTERNAL_NO1   0x2221 /**< \brief Continuous over current no 1*/
//#define ERROR_CONTINUOUS_OVER_CURRENT_DEVICE_INTERNAL_NO2   0x2222 /**< \brief Continuous over current no 2*/
//#define ERROR_SHORT_CIRCUIT_EARTH_LEAKAGE_DEVICE_INTERNAL   0x2230 /**< \brief Short circuit/earth leakage (device internal)*/
//#define ERROR_EARTH_LEAKAGE_DEVICE_INTERNAL                 0x2240 /**< \brief Earth leakage (device internal)*/
//#define ERROR_SHORT_CIRCUIT_DEVICE_INTERNAL                 0x2250 /**< \brief Short circuit (device internal)*/
//#define ERROR_CONTINUOUS_OVER_CURRENT                       0x2310 /**< \brief Continuous over current*/
//#define ERROR_CONTINUOUS_OVER_CURRENT_NO1                   0x2311 /**< \brief Continuous over current no 1*/
//#define ERROR_CONTINUOUS_OVER_CURRENT_NO2                   0x2312 /**< \brief Continuous over current no 2*/
//#define ERROR_SHORT_CIRCUIT_EARTH_LEAKAGE_MOTOR_SIDE        0x2320 /**< \brief Short circuit/earth leakage (motor-side)*/
//#define ERROR_EARTH_LEAKAGE_MOTOR_SIDE                      0x2330 /**< \brief Earth leakage (motor-side)*/
//#define ERROR_EARTH_LEAKAGE_PHASE_U                         0x2331 /**< \brief Earth leakage phase U*/
//#define ERROR_EARTH_LEAKAGE_PHASE_V                         0x2332 /**< \brief Earth leakage phase V*/
//#define ERROR_EARTH_LEAKAGE_PHASE_W                         0x2333 /**< \brief Earth leakage phase W*/
//#define ERROR_SHORT_CIRCUIT_MOTOR_SIDE                      0x2340 /**< \brief Short circuit (motor-side)*/
//#define ERROR_SHORT_CIRCUIT_PHASES_U_V                      0x2341 /**< \brief Short circuit phases U-V*/
//#define ERROR_EARTH_LEAKAGE_PHASE_V_W                       0x2342 /**< \brief Earth leakage phase V-W*/
//#define ERROR_EARTH_LEAKAGE_PHASE_W_U                       0x2343 /**< \brief Earth leakage phase W-U*/
//#define ERROR_LOAD_LEVEL_FAULT_I2T_THERMAL_STATE            0x2350 /**< \brief Load level fault (I2t, thermal state)*/
//#define ERROR_LOAD_LEVEL_WARNING_I2T_THERMAL_STATE          0x2351 /**< \brief Load level warning (I2t, thermal state)*/
//#define ERROR_MAINS_OVER_VOLTAGE                            0x3110 /**< \brief Mains over-voltage*/
//#define ERROR_MAINS_OVER_VOLTAGE_PHASE_L1                   0x3111 /**< \brief Mains over-voltage phase L1*/
//#define ERROR_MAINS_OVER_VOLTAGE_PHASE_L2                   0x3112 /**< \brief Mains over-voltage phase L2 */
//#define ERROR_MAINS_OVER_VOLTAGE_PHASE_L3                   0x3113 /**< \brief Mains over-voltage phase L3*/
//#define ERROR_MAINS_UNDER_VOLTAGE                           0x3120 /**< \brief Mains under-voltage*/
//#define ERROR_MAINS_UNDER_VOLTAGE_PHASE_L1                  0x3121 /**< \brief Mains under-voltage phase L1*/
//#define ERROR_MAINS_UNDER_VOLTAGE_PHASE_L2                  0x3122 /**< \brief Mains under-voltage phase L2*/
//#define ERROR_MAINS_UNDER_VOLTAGE_PHASE_L3                  0x3123 /**< \brief Mains under-voltage phase L3*/
//#define ERROR_PHASE_FAILURE                                 0x3130 /**< \brief Phase failure*/
//#define ERROR_PHASE_FAILURE_L1                              0x3131 /**< \brief Phase failure L1*/
//#define ERROR_PHASE_FAILURE_L2                              0x3132 /**< \brief Phase failure L2*/
//#define ERROR_PHASE_FAILURE_L3                              0x3133 /**< \brief Phase failure L3*/
//#define ERROR_PHASE_SEQUENCE                                0x3134 /**< \brief Phase sequence*/
//#define ERROR_MAINS_FREQUENCY                               0x3140 /**< \brief Mains frequency*/
//#define ERROR_MAINS_FREQUENCY_TOO_GREAT                     0x3141 /**< \brief Mains frequency too great*/
//#define ERROR_MAINS_FREQUENCY_TOO_SMALL                     0x3142 /**< \brief Mains frequency too small*/
//#define ERROR_DC_LINK_OVER_VOLTAGE                          0x3210 /**< \brief DC link over-voltage*/
//#define ERROR_OVER_VOLTAGE_NO_1                             0x3211 /**< \brief Over-voltage no  1*/
//#define ERROR_OVER_VOLTAGE_NO_2                             0x3212 /**< \brief Over voltage no  2 */
//#define ERROR_DC_LINK_UNDER_VOLTAGE                         0x3220 /**< \brief DC link under-voltage*/
//#define ERROR_UNDER_VOLTAGE_NO_1                            0x3221 /**< \brief Under-voltage no  1*/
//#define ERROR_UNDER_VOLTAGE_NO_2                            0x3222 /**< \brief Under-voltage no  2*/
//#define ERROR_LOAD_ERROR                                    0x3230 /**< \brief Load error*/
//#define ERROR_OUTPUT_OVER_VOLTAGE                           0x3310 /**< \brief Output over-voltage*/
//#define ERROR_OUTPUT_OVER_VOLTAGE_PHASE_U                   0x3311 /**< \brief Output over-voltage phase U*/
//#define ERROR_OUTPUT_OVER_VOLTAGE_PHASE_V                   0x3312 /**< \brief Output over-voltage phase V*/
//#define ERROR_OUTPUT_OVER_VOLTAGE_PHASE_W                   0x3313 /**< \brief Output over-voltage phase W*/
//#define ERROR_ARMATURE_CIRCUIT                              0x3320 /**< \brief Armature circuit*/
//#define ERROR_ARMATURE_CIRCUIT_INTERRUPTED                  0x3321 /**< \brief Armature circuit interrupted*/
//#define ERROR_FIELD_CIRCUIT                                 0x3330 /**< \brief Field circuit error */
//#define ERROR_FIELD_CIRCUIT_INTERRUPTED                     0x3331 /**< \brief Field circuit interrupted*/
//#define ERROR_EXCESS_AMBIENT_TEMPERATURE                    0x4110 /**< \brief Excess ambient temperature*/
//#define ERROR_TOO_LOW_AMBIENT_TEMPERATURE                   0x4120 /**< \brief Too low ambient temperature*/
//#define ERROR_TEMPERATURE_SUPPLY_AIR                        0x4130 /**< \brief Temperature supply air*/
//#define ERROR_TEMPERATURE_AIR_OUTLET                        0x4140 /**< \brief Temperature air outlet*/
//#define ERROR_EXCESS_TEMPERATURE_DEVICE                     0x4210 /**< \brief Excess temperature device*/
//#define ERROR_TOO_LOW_TEMPERATURE_DEVICE                    0x4220 /**< \brief Too low temperature device*/
//#define ERROR_TEMPERATURE_DRIVE                             0x4300 /**< \brief Temperature drive error*/
//#define ERROR_EXCESS_TEMPERATURE_DRIVE                      0x4310 /**< \brief Excess temperature drive error*/
//#define ERROR_TOO_LOW_TEMPERATURE_DRIVE                     0x4320 /**< \brief Too low temperature drive error*/
//#define ERROR_TEMPERATURE_SUPPLY                            0x4400 /**< \brief Temperature supply error*/
//#define ERROR_EXCESS_TEMPERATURE_SUPPLY                     0x4410 /**< \brief Excess temperature supply*/
//#define ERROR_TOO_LOW_TEMPERATURE_SUPPLY                    0x4420 /**< \brief Too low temperature supply*/
//#define ERROR_SUPPLY_ERROR                                  0x5100 /**< \brief Supply error*/
//#define ERROR_SUPPLY_LOW_VOLTAGE                            0x5110 /**< \brief Supply low voltage*/
//#define ERROR_U1_SUPPLY_15V                                 0x5111 /**< \brief U1 = supply +15V/-15V*/
//#define ERROR_U2_SUPPLY_24_V                                0x5112 /**< \brief U2 = supply +24 V*/
//#define ERROR_U3_SUPPLY_5_V                                 0x5113 /**< \brief U3 = supply +5 V*/
//#define ERROR_U4_MANUFACTURER_SPECIFIC                      0x5114 /**< \brief U4 = manufacturer-specific error*/
//#define ERROR_U5_MANUFACTURER_SPECIFIC                      0x5115 /**< \brief U5 = manufacturer-specific error*/
//#define ERROR_U6_MANUFACTURER_SPECIFIC                      0x5116 /**< \brief U6 = manufacturer-specific error*/
//#define ERROR_U7_MANUFACTURER_SPECIFIC                      0x5117 /**< \brief U7 = manufacturer-specific error*/
//#define ERROR_U8_MANUFACTURER_SPECIFIC                      0x5118 /**< \brief U8 = manufacturer-specific error*/
//#define ERROR_U9_MANUFACTURER_SPECIFIC                      0x5119 /**< \brief U9 = manufacturer-specific error*/
//#define ERROR_SUPPLY_INTERMEDIATE_CIRCUIT                   0x5120 /**< \brief Supply intermediate circuit*/
////#define ERROR_CONTROL                                     0x5200
//#define ERROR_CONTROL_MEASUREMENT_CIRCUIT                   0x5210 /**< \brief Measurement circuit*/
//#define ERROR_CONTROL_COMPUTING_CIRCUIT                     0x5220 /**< \brief Computing circuit*/
//#define ERROR_OPERATING_UNIT                                0x5300 /**< \brief Operating unit error*/
//#define ERROR_POWER_SECTION                                 0x5400 /**< \brief Power section error*/
//#define ERROR_OUTPUT_STAGES                                 0x5410 /**< \brief Output stages error*/
//#define ERROR_CHOPPER                                       0x5420 /**< \brief Chopper error*/
//#define ERROR_INPUT_STAGES                                  0x5430 /**< \brief Input stages error*/
//#define ERROR_CONTACTS_ERROR                                0x5440 /**< \brief Contacts error*/
//#define ERROR_CONTACT_1_MANUFACTURER_SPECIFIC               0x5441 /**< \brief Contact 1 = manufacturer-specific error*/
//#define ERROR_CONTACT_2_MANUFACTURER_SPECIFIC               0x5442 /**< \brief Contact 2 = manufacturer-specific error*/
//#define ERROR_CONTACT_3_MANUFACTURER_SPECIFIC               0x5443 /**< \brief Contact 3 = manufacturer-specific error*/
//#define ERROR_CONTACT_4_MANUFACTURER_SPECIFIC               0x5444 /**< \brief Contact 4 = manufacturer-specific error*/
//#define ERROR_CONTACT_5_MANUFACTURER_SPECIFIC               0x5445 /**< \brief Contact 5 = manufacturer-specific error*/
//#define ERROR_FUSES_ERROR                                   0x5450 /**< \brief Fuses error*/
//#define ERROR_S1_L1                                         0x5451 /**< \brief S1 = l1 error*/
//#define ERROR_S2_L2                                         0x5452 /**< \brief S2 = l2 error*/
//#define ERROR_S3_L3                                         0x5453 /**< \brief S3 = l3 error*/
//#define ERROR_S4_MANUFACTURER_SPECIFIC                      0x5454 /**< \brief S4 = manufacturer-specific error*/
//#define ERROR_S5_MANUFACTURER_SPECIFIC                      0x5455 /**< \brief S5 = manufacturer-specific error*/
//#define ERROR_S6_MANUFACTURER_SPECIFIC                      0x5456 /**< \brief S6 = manufacturer-specific error*/
//#define ERROR_S7_MANUFACTURER_SPECIFIC                      0x5457 /**< \brief S7 = manufacturer-specific error*/
//#define ERROR_S8_MANUFACTURER_SPECIFIC                      0x5458 /**< \brief S8 = manufacturer-specific error*/
//#define ERROR_S9_MANUFACTURER_SPECIFIC                      0x5459 /**< \brief S9 = manufacturer-specific error*/
//#define ERROR_HARDWARE_MEMORY                               0x5500 /**< \brief Hardware memory error*/
//#define ERROR_RAM                                           0x5510 /**< \brief RAM error*/
//#define ERROR_ROM_EPROM                                     0x5520 /**< \brief ROM/EPROM error*/
//#define ERROR_EEPROM                                        0x5530 /**< \brief EEPROM error*/
//#define ERROR_SOFTWARE_RESET_WATCHDOG                       0x6010 /**< \brief Software reset (watchdog)*/
////0x6301_TO_0x630F        ERROR_DATA_RECORD_NO_1_TO_NO_15
//#define ERROR_LOSS_OF_PARAMETERS                            0x6310 /**< \brief Loss of parameters*/
//#define ERROR_PARAMETER_ERROR                               0x6320 /**< \brief Parameter error*/
//#define ERROR_POWER_ERROR                                   0x7100 /**< \brief Power error*/
//#define ERROR_BRAKE_CHOPPER                                 0x7110 /**< \brief Brake chopper*/
//#define ERROR_FAILURE_BRAKE_CHOPPER                         0x7111 /**< \brief Failure brake chopper*/
//#define ERROR_OVER_CURRENT_BRAKE_CHOPPER                    0x7112 /**< \brief Over current brake chopper*/
//#define ERROR_PROTECTIVE_CIRCUIT_BRAKE_CHOPPER              0x7113 /**< \brief Protective circuit brake chopper error*/
//#define ERROR_MOTOR_ERROR                                   0x7120 /**< \brief Motor error*/
//#define ERROR_MOTOR_BLOCKED                                 0x7121 /**< \brief Motor blocked error*/
//#define ERROR_MOTOR_ERROR_OR_COMMUTATION_MALFUNC            0x7122 /**< \brief Motor error or commutation malfunc */
//#define ERROR_MOTOR_TILTED                                  0x7123 /**< \brief Motor tilted*/
//#define ERROR_MEASUREMENT_CIRCUIT                           0x7200 /**< \brief Measurement circuit*/
//#define ERROR_SENSOR_ERROR                                  0x7300 /**< \brief Sensor error*/
//#define ERROR_TACHO_FAULT                                   0x7301 /**< \brief Tacho fault*/
//#define ERROR_TACHO_WRONG_POLARITY                          0x7302 /**< \brief Tacho wrong polarity*/
//#define ERROR_RESOLVER_1_FAULT                              0x7303 /**< \brief Resolver 1 fault*/
//#define ERROR_RESOLVER_2_FAULT                              0x7304 /**< \brief Resolver 2 fault*/
//#define ERROR_INCREMENTAL_SENSOR_1_FAULT                    0x7305 /**< \brief Incremental sensor 1 fault*/
//#define ERROR_INCREMENTAL_SENSOR_2_FAULT                    0x7306 /**< \brief Incremental sensor 2 fault*/
//#define ERROR_INCREMENTAL_SENSOR_3_FAULT                    0x7307 /**< \brief Incremental sensor 3 fault*/
//#define ERROR_SPEED                                         0x7310 /**< \brief Speed error*/
//#define ERROR_POSITION                                      0x7320 /**< \brief Position error*/
//#define ERROR_COMPUTATION_CIRCUIT                           0x7400 /**< \brief Computation circuit*/
//#define ERROR_COMMUNICATION                                 0x7500 /**< \brief Communication error*/
//#define ERROR_SERIAL_INTERFACE_NO_1                         0x7510 /**< \brief Serial interface no  1 error*/
//#define ERROR_SERIAL_INTERFACE_NO_2                         0x7520 /**< \brief Serial interface no  2 error*/
//#define ERROR_DATA_STORAGE_EXTERNAL                         0x7600 /**< \brief Data storage (external) error*/
//#define ERROR_TORQUE_CONTROL                                0x8300 /**< \brief Torque control error*/
//#define ERROR_EXCESS_TORQUE                                 0x8311 /**< \brief Excess torque error*/
//#define ERROR_DIFFICULT_START_UP                            0x8312 /**< \brief Difficult start up error*/
//#define ERROR_STANDSTILL_TORQUE                             0x8313 /**< \brief Standstill torque error*/
//#define ERROR_INSUFFICIENT_TORQUE                           0x8321 /**< \brief Insufficient torque error*/
//#define ERROR_TORQUE_FAULT                                  0x8331 /**< \brief Torque fault*/
//#define ERROR_VELOCITY_SPEED_CONTROLLER                     0x8400 /**< \brief Velocity speed controller*/
//#define ERROR_POSITION_CONTROLLER                           0x8500 /**< \brief Position controller*/
//#define ERROR_POSITIONING_CONTROLLER                        0x8600 /**< \brief Positioning controller*/
//#define ERROR_FOLLOWING_ERROR                               0x8611 /**< \brief Following error*/
//#define ERROR_REFERENCE_LIMIT                               0x8612 /**< \brief Reference limit*/
//#define ERROR_SYNC_CONTROLLER                               0x8700 /**< \brief Sync controller*/
//#define ERROR_WINDING_CONTROLLER                            0x8800 /**< \brief Winding controller*/
//#define ERROR_PROCESS_DATA_MONITORING                       0x8900 /**< \brief Process data monitoring*/
////#define ERROR_CONTROL                                     0x8A00
//#define ERROR_DECELERATION                                  0xF001 /**< \brief Deceleration error*/
//#define ERROR_SUB_SYNCHRONOUS_RUN                           0xF002 /**< \brief Sub-synchronous run error*/
//#define ERROR_STROKE_OPERATION                              0xF003 /**< \brief Stroke operation error*/
//#define ERROR_CONTROL                                     0xF004
//0xFF00_TO_0xFFFF        MANUFACTURER_SPECIFIC                   


/*---------------------------------------------
-    CiA402 generic error option code values
        Note: Not all values are valid for each error option code.
        A detailed description of the option code values are listed in the specification IEC 61800-7-200
        0x605B    : action in state transition 8
        0x605C    : action in state transition 5
-----------------------------------------------*/
#define DISABLE_DRIVE                    0 /**< \brief Disable drive (options: 0x605B; 0x605C; 0x605E)*/
#define SLOW_DOWN_RAMP                   1 /**< \brief Slow down ramp (options: 0x605B; 0x605C; 0x605E)*/
#define QUICKSTOP_RAMP                   2 /**< \brief Quick stop ramp (options: 0x605E)*/
#define STOP_ON_CURRENT_LIMIT            3 /**< \brief Stop on current limit (options: 0x605E)*/
#define STOP_ON_VOLTAGE_LIMIT            4 /**< \brief Stop on voltage limit (options: 0x605E)*/

/*---------------------------------------------
-    Specific values for Quick stop option code (object 0x605A) (IEC61800_184e)
        indicated the quick stop function
-----------------------------------------------*/
//-32768 to -1        MANUFACTURER_SPECIFIC
#define SLOWDOWN_RAMP_NO_TRANSIT                5 /**< \brief Slow down on slow down ramp and stay in Quick Stop Active*/
#define QUICKSTOP_RAMP_NO_TRANSIT               6 /**< \brief Slow down on quick stop ramp and stay in Quick Stop Active*/
#define CURRENT_LIMIT_NO_TRANSIT                7 /**< \brief Slow down on current limit and stay in Quick Stop Active*/
#define VOLTAGE_LIMIT_NO_TRANSIT                8 /**< \brief Slow down on voltage limit and stay in Quick Stop Active*/
//9 to 32767        RESERVED

/*---------------------------------------------
-    Module Identifications (each supported operation mode has a module)
-----------------------------------------------*/
#define CSV_CSP_MODULE_ID       0x00119800 /**< \brief Module Id for cyclic synchronous position/velocity mode (dynamic switching supported)*/
#define CSP_MODULE_ID           0x00219800 /**< \brief Module Id for cyclic synchronous position mode*/
#define CSV_MODULE_ID           0x00319800 /**< \brief Module Id for cyclic synchronous velocity mode*/
#define PP_MODULE_ID            0x00419800 /**< \brief Module Id for profile position mode*/
#define HM_MODULE_ID            0x00519800 /**< \brief Module Id for homing mode*/


/** \brief 0x607D (Software Position Limit) data structure*/
typedef struct  {
   UINT16    u16SubIndex0; /**< \brief SubIndex 0*/
   INT32    i32MinLimit; /**< \brief Minimum limit*/
   INT32    i32MaxLimit; /**< \brief Maximum limit*/
}
TOBJ607D;

/** \brief 0x6099 (Homing Speeds) data structure*/
typedef struct  {
   UINT16     u16SubIndex0; /**< \brief SubIndex 0*/
   UINT32    i32Speedforswitch; /**< \brief Speed during search for switch*/
   UINT32    i32Speedforzero;
}
TOBJ6099;

/** \brief 0x6091 (Gear ratio) data structure*/
typedef struct  {
   UINT16   u16SubIndex0; /**< \brief SubIndex 0*/
   UINT32    MotorRevolutions;
   UINT32    ShaftRevolutions;
}
TOBJ6091;

///** \brief 0x6093 (Position factor) data structure*/
//typedef struct  {
//   UINT16   u16SubIndex0; /**< \brief SubIndex 0*/
//   UINT32    numerator;
//   UINT32    divisor;
//}
//
//TOBJ6093;
///** \brief 0x6094 (Velocity factor) data structure*/
//typedef struct  {
//   UINT16   u16SubIndex0; /**< \brief SubIndex 0*/
//   UINT32    numerator;
//   UINT32    divisor;
//}
//TOBJ6094;
///** \brief 0x6097 (Acceleration factor) data structure*/
//typedef struct  {
//   UINT16   u16SubIndex0; /**< \brief SubIndex 0*/
//   UINT32    numerator;
//   UINT32    divisor;
//}
//TOBJ6097;

/** \brief Object 0x60C1 (Interpolation Data Record) data structure*/
typedef struct  {
   UINT16   u16SubIndex0; /**< \brief SubIndex 0*/
   INT32   u32Interpolationdatarecord; /**< \brief Interpolation data record*/
}
TOBJ60C1;
/** \brief Object 0x60C2 (Interpolation Time Period) data structure*/
typedef struct  {
   UINT16   u16SubIndex0; /**< \brief SubIndex 0*/
   UINT8    u8InterpolationPeriod; /**< \brief Interpolation period*/
   INT8     i8InterpolationIndex; /**< \brief Interpolation index*/
}
TOBJ60C2;

/** \brief Data structure to handle the axis specific object data*/
typedef struct
{
    UINT16   objErrorCode; /**< \brief Error Code (0x603F)*/
    UINT16   objControlWord; /**< \brief Control Word (0x6040)*/
    UINT16   objStatusWord; /**< \brief Status Word (0x6041)*/
    INT16    objQuickStopOptionCode; /**< \brief Quick Stop Option Code (0x605A)*/
    
    INT16    objShutdownOptionCode; /**< \brief Shutdown Option Code (0x605B)*/
    INT16    objDisableOptionCode; /**< \brief Disable operation option code (0x605C)*/
    INT16    objHaltOptionCode; /**< \brief Halt Option Code (0x605D)*/
    INT16    objFaultOptionCode;/**< \brief Fault Option Code (0x605E)*/
    
    INT16    rsd605F_1;
    INT16    rsd605F_2;
    INT8     objModesOfOperation; /**< \brief Modes of Operation (0x6060)*/
    INT8     rsd1; 
    INT8     objModesOfOperationDisplay; /**< \brief Mode of Operation Display (0x6061)*/
    INT8     rsd2;
    
    INT32    objPositionDemandValue;/**breaf Position Demand Walue(0x6062)*/
    INT32    objPositionActualInterValue;/**breaf Position Actual Internal Value(0x6063)*/
    INT32    objPositionActualValue; /**< \brief Position Actual Value (0x6064)*/
    UINT32   objFollowingErrorWindow;/**breaf Following Error Window(0x6065)*/
    
    UINT16   objFollowErrorTimeOut;/**breaf Follow Error Time Out(0x6066)*/
    UINT16   rsd3;
    UINT32   objPositionWindow;/**breaf Position Window(0x6067)*/
    
    UINT16   objPositionWindowTime;/**breaf Position Window Time(0x6068)*/
    UINT16   rsd4;
    UINT16   rsd6069_1;
    UINT16   rsd6069_2;
    
    INT32    objVelocityDemandValue;/**breaf Velocity Demand Value(0x606B)*/
    INT32    objVelocityActualValue; /**< \brief Actual Velocity Value (0x606C)*/
    
    UINT16   objVelocityWindow;/**brief Velocity Window(0x606D)*/
    UINT16   objVelocityWindowTime;/**brief Velocity Window Time(0x606E)*/
    UINT16   objVelocitythreshold; /**brief Velocity threshold(0x606F)*/
    UINT16   objVelocitythresholdTime;/**brief Velocity threshold time(0x6070)*/   //
    
    INT16    objTargetTorque;/**brief Target Torque(0x6071)*/
    UINT16   objMaxTorque;   /**brief Max Torque(0x6072)*/
    UINT16   rsd6073_1;      
    UINT16   rsd6073_2;      
    
    INT16    objTorqueDemandValue;/**brief torque Demand Value(0x6074)*/
    UINT16   rsd5;
    UINT32   objMotorRatedCurrent;/**brief Motor Rated Current(0x6075)*/        //
    
    UINT32   objMotorRatedTorque;/**brief Motor Rated Torque(0x6076)*/          //
    INT16    objTorqueActualValue;  /**< \brief Torque Actual Value (0x6077)*/
    INT16    objCurrentActualValue; /**< \brief Current Actual Value (0x6078)*/
    
    UINT32   objDCLinkActualValue;  /**< \brief DC Link Circuit Value (0x6079)*/
    INT32    objTargetPosition; /**< \brief Target Position (0x607A)*/
    
    UINT16   rsd607B_1;
    UINT16   rsd607B_2;
    INT32    objHomeOffset;/**brief Home Offset(0x607C)*/
    
    TOBJ607D objSoftwarePositionLimit; /**< \brief Software Position limit (0x607D)*/
    UINT16   rsd6;
    UINT16   objPolarity;/**brief Polarity(0x607E)*/
    
    UINT32   objMaxProfileVelocity;/**brief Max Profile Velocity(0x607F)*/
    UINT32   objMaxMotorSpeed;/**brief Max Motor Speed(0x6080)*/
    UINT32   objProfileVelocity;/**brief Profile Velocity(0x6081)*/
    UINT16   rsd6082_1;
    UINT16   rsd6082_2;
    
    UINT32   objProfileAcceleration;/**brief Profile Acceleration(0x6083)*/
    UINT32   objProfileDeceleration;/**brief Profile Deceleration(0x6084)*/
    UINT32   objQuickStopDeclaration; /**< \brief Quick Stop Declaration (0x6085)*/
    UINT16   rsd6086_1;
    UINT16   rsd6086_2;
    
    UINT32   objTorqueSlop;/**brief Torque Slop(0x6087)*/
    TOBJ6091 objGearRatio; /**brief Gear Ratio (0x6091)*/
    
    INT8     objHomeMethod;/**brief Home Method(0x6098)*/
    UINT8    rsd7;
    UINT8    rsd8;
    UINT8    rsd9;
    TOBJ6099 objHomingSpeeds;/*brief Homing Speeds(0x6099)*/
    UINT32   objHomingAcceleration;/**brief Homing Acceleration(0x609A)*/
    
    UINT16   rsd609B_1;
    UINT16   rsd609B_2;
    UINT16   rsd609C_1;
    UINT16   rsd609C_2;
    
    INT32    objPositionoffset; /**brief Position Offset(0x60B0)*/
    INT32    objVelocityOffset;/**brief Velocity Offset(0x60B1)*/
    INT16    objTorqueOffset;/**brief Torque Offset(0x60B2)*/
    UINT16   rsd10;

    UINT16   objTouchProbeFunction;/**brief Touch Probe Function(0x60B8)*/
    UINT16   objTouchProbeStatus;/**brief Touch Probe Status(0x60B9)*/
    INT32    objTP1PositiveEPValue;/**brief Touch Probe 1 Positive Edge Position Value(0x60BA)*/
    INT32    objTP1NegativeEPValue;/**brief Touch Probe 1 Negative Edge Position Value(0x60BB)*/
    INT32    objTP2PositiveEPValue;/**brief Touch Probe 1 Positive Edge Position Value(0x60BC)*/
    INT32    objTP2NegativeEPValue;/**brief Touch Probe 1 Negative Edge Position Value(0x60BD)*/

    TOBJ60C1 objInterpolationDataRecord;/**brief Interpolation Data Record(0x60C1)*/
    TOBJ60C2 objInterpolationTimePeriod; /**< \brief Interpolation Time Period (0x60C2)*/
    UINT32   objMaxAcceleration;/**brief Max acceleration(0x60C5)*/
    UINT32   ojbMaxDeceleration;/**brief Max deceleration(0x60C6)*/
    
    UINT16   objPosTqLimitValue;/**brief Positive Torque Limit Value(0x60E0)*/
    UINT16   objNegTqLimitValue;/**brief Negative Torque Limit Value(0x60E1)*/
    INT32    objFollowingErrorActualValue;/**brief Following Error Actual Value(0x60F4)*/
    INT32    objPositionDemandInterValue;/**brief Position Demand Internal Value(0x60FC)*/
    UINT32   objDigitalInputs;/**brief Digital Inputs(0x60FD)*/
    UINT32   objDigitalOutput;/**brief Digital Output(0x60FE)*/
    INT32    objTargetVelocity; /**< \brief Target Velocity (0x60FF)*/
    UINT32   objSupportedDriveModes; /**< \brief Supported Drive Modes (0x6502)*/
}
CiA402Objects;

/**
 * 对象字典的属性与最大值最小值，在需要的情况下写入
 */
// 功能码属性中参数是否显示
typedef enum
{
	OBJ_USIGN = 0,							//无符号数
	OBJ_SIGN  = 1							//有符号数
}ENUM_OBJSign;//数据符号

// 定义功能代码的属性
typedef union
{
	UINT32 U;		//无符号数
	INT32  S;		//有符号数
}TYPE_ObjSign;

typedef struct
{
	UINT16 Index;     //索引
	UINT8  subIndex;  //子索引
	UINT8  Sign;      //有无符号
    UINT8  rsd1;
	UINT8  rsd2;      //保留
	TYPE_ObjSign Max; //最大值
	TYPE_ObjSign Min; //最小值
}Obj_Attribute;

/** \brief Data structure to handle an Cia402 axis*/
typedef struct
{
    BOOL        bAxisIsActive; /**< \brief Indicates if active is active*/
    BOOL        bAxisInitReady;//驱动器初始化完成
    BOOL        bAxisFunctionEnabled;//使能驱动器
    BOOL        rsd1;
    UINT16      i16State; /**< \brief Axis state*/
    UINT16      CmdSource;//表示的是当前控制命令的来源，0表示内部命令，1表示外部命令，用于标注状态字Remote位。
    UINT16      u16PendingOptionCode; /**< \brief Pending operation code*/
    UINT16      u16PendState;         /**< \brief Pending*/
    UINT16      EcatStopMode;           //停止方式
    UINT16      EcatDbFlag;             //动态制动标志
    UINT16      OffStop_FaultStop;      //ECAT伺服中故障时的停车方式
	UINT16      OffStop_QuickStop;      //ECAT伺服中Qick Stop停机方式
	UINT16      OffStop_HaltStop;       //ECAT伺服中Halt停机方式:等于1表示慢速停车，为2表示快速停车
	UINT16      OffStop_Other;          //其他模块发出的停车方式
	UINT16      LastControlWord6040;    //上次操作的控制字
	UINT16      LastFaultRest;
	UINT16      StatusWord;
    UINT16      InitReady;//准备好
	double      Encoderres;             //编码器分辨率
    double      SpdAccDecPu;           /**< \brief Current position within control loop*/
    double      PosAccDecPu;              /**< \brief Current position within control loop*/
    double      SpdCmdPU;//速度给定标幺值
    double      SpdFbPu; 
    double      PosPU;//位置环标幺值
    float       TorquePU;//
    UINT16      TpN;//在中断时间内要执行多少次位置环更新             DrvCoeff->Tpwm;
    UINT16      TprwTimes;//当前已经执行了多少次；
    UINT32      u32CycleTime; /**< \brief Motion controller cycletime in us*/
    INT32       objTargetLastPosition;
    UINT32      LocationTarget_limit;
    CiA402Objects        Objects; /**< \brief CiA402 Axis object variable*/
    TOBJECT OBJMEM      * ObjDic; /**< \brief Pointer to Axis specific object dictionary*/
}
TCiA402Axis, *pTCiA402Axis;
/** @}*/

#endif

/*---------------------------------------------
-    Entry Descriptions and object names
-----------------------------------------------*/
#ifdef _CiA402_	/*online时此处定义名称有效*/

#if SERVOTYPE == SERVO_ETHERCAT

/**
 * \addtogroup CiA402Objects CiA402 Axis Objects
 * @{
 */

/** \brief Object 0x603F (Error Code) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x603F = {DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ|OBJACCESS_TXPDOMAPPING};
/** \brief Object 0x603F (Error Code) object name*/
OBJCONST UCHAR OBJMEM aName0x603F[] = "Error Code";

/** \brief Object 0x6040 (Control Word) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6040 = {DEFTYPE_UNSIGNED16, 0x10, (ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING)};
/** \brief Object 0x6040 (Control Word) object name*/
OBJCONST UCHAR OBJMEM aName0x6040[] = "Control Word";

/** \brief Object 0x6041 (Status Word) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6041 = {DEFTYPE_UNSIGNED16, 0x10, (ACCESS_READ|OBJACCESS_TXPDOMAPPING)};
/** \brief Object 0x6041 (Status Word) object name*/
OBJCONST UCHAR OBJMEM aName0x6041[] = "Status Word";

/** \brief Object 0x605A (Quickstop Option Code) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x605A = {DEFTYPE_INTEGER16, 0x10, ACCESS_READWRITE};
/** \brief Object 0x605A (Quickstop Option Code) object name*/
OBJCONST UCHAR OBJMEM aName0x605A[] = "Quickstop Option Code";

/** \brief Object 0x605B (Shutdown Option Code) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x605B = {DEFTYPE_INTEGER16, 0x10, ACCESS_READWRITE};
/** \brief Object 0x605B (Shutdown Option Code) object name*/
OBJCONST UCHAR OBJMEM aName0x605B[] = "Shutdown Option Code";

/** \brief Object 0x605C (Disable operation option code) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x605C = {DEFTYPE_INTEGER16, 0x10, ACCESS_READWRITE};
/** \brief Object 0x605C (Disable operation option code) object name*/
OBJCONST UCHAR OBJMEM aName0x605C[] = "Disable Operation Option Code";

/** \brief Object 0x605D (Halt Option Code) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x605D = {DEFTYPE_INTEGER16, 0x10, ACCESS_READWRITE};
/** \brief Object 0x605D (Halt Option Code) object name*/
OBJCONST UCHAR OBJMEM aName0x605D[] = "Halt Option Code";

/** \brief Object 0x605E (Fault Reaction Option Code) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x605E = {DEFTYPE_INTEGER16, 0x10, ACCESS_READWRITE};
/** \brief Object 0x605E (Fault Reaction Option Code) object name*/
OBJCONST UCHAR OBJMEM aName0x605E[] = "Fault Reaction Option Code";

/** \brief Object 0x6060 (Modes of Operation) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6060 = {DEFTYPE_INTEGER8, 0x08, (ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING)};
/** \brief Object 0x6060 (Modes of Operation) object name*/
OBJCONST UCHAR OBJMEM aName0x6060[] = "Modes of Operation";

/** \brief Object 0x6061 (Modes of Operation Display) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6061 = {DEFTYPE_INTEGER8, 0x08, (ACCESS_READ|OBJACCESS_TXPDOMAPPING)};
/** \brief Object 0x6061 (Modes of Operation Display) object name*/
OBJCONST UCHAR OBJMEM aName0x6061[] = "Modes of Operation Display";

/** \brief Object 0x6062 (Position Demand Value) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6062 = {DEFTYPE_INTEGER32, 0x020, (ACCESS_READ|OBJACCESS_TXPDOMAPPING)};
/** \brief Object 0x6062 (Position Demand Value) object name*/
OBJCONST UCHAR OBJMEM aName0x6062[] = "Position Demand Value";

/** \brief Object 0x6063 (Position Actual Internal Value) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6063 = {DEFTYPE_INTEGER32, 0x020, (ACCESS_READ|OBJACCESS_TXPDOMAPPING)};
/** \brief Object 0x6063 (Position Actual Internal Value) object name*/
OBJCONST UCHAR OBJMEM aName0x6063[] = "Position Actual Internal Value";

/** \brief Object 0x6064 (Position Actual Value) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6064 = {DEFTYPE_INTEGER32, 0x20, (ACCESS_READ|OBJACCESS_TXPDOMAPPING)};
/** \brief Object 0x6064 (Position Actual Value) object name*/
OBJCONST UCHAR OBJMEM aName0x6064[] = "Position Actual Value";

/** \brief Object 0x6065 (Following Error Window) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6065 = {DEFTYPE_UNSIGNED32, 0x20, (ACCESS_READWRITE|OBJACCESS_NOPDOMAPPING)};
/** \brief Object 0x6065 (Following Error Window) object name*/
OBJCONST UCHAR OBJMEM aName0x6065[] = "Following Error Window";

/** \brief Object 0x6066 (Following Error Time Out) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6066 = {DEFTYPE_UNSIGNED16, 0x10, (ACCESS_READWRITE|OBJACCESS_NOPDOMAPPING)};
/** \brief Object 0x6066 (Following Error Time Out) object name*/
OBJCONST UCHAR OBJMEM aName0x6066[] = "Following Error Time Out";

/** \brief Object 0x6067 (Position Window) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6067 = {DEFTYPE_UNSIGNED32, 0x20, (ACCESS_READWRITE|OBJACCESS_NOPDOMAPPING)};
/** \brief Object 0x6067 (Position Window) object name*/
OBJCONST UCHAR OBJMEM aName0x6067[] = "Position Window";

/** \brief Object 0x6068 (Positin Window Time) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6068 = {DEFTYPE_UNSIGNED16, 0x10, (ACCESS_READWRITE|OBJACCESS_NOPDOMAPPING)};
/** \brief Object 0x6068 (Positin Window Time) object name*/
OBJCONST UCHAR OBJMEM aName0x6068[] = "Position Window Time";

/** \brief Object 0x606B (Velocity Demand Value) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x606B = {DEFTYPE_INTEGER32, 0x20, (ACCESS_READ|OBJACCESS_TXPDOMAPPING)};
/** \brief Object 0x606B (Velocity Demand Value) object name*/
OBJCONST UCHAR OBJMEM aName0x606B[] = "Velocity Demand Value";

/** \brief Object 0x606C (Velocity Actual Value) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x606C = {DEFTYPE_INTEGER32, 0x20, (ACCESS_READ|OBJACCESS_TXPDOMAPPING)};

/** \brief Object 0x606C (Velocity Actual Value) object name*/
OBJCONST UCHAR OBJMEM aName0x606C[] = "Velocity Actual Value";

/** \brief Object 0x606D (Velocity Window) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x606D = {DEFTYPE_UNSIGNED16, 0x10, (ACCESS_READWRITE|OBJACCESS_NOPDOMAPPING)};
/** \brief Object 0x606D (Velocity Window) object name*/
OBJCONST UCHAR OBJMEM aName0x606D[] = "Velocity Window";

/** \brief Object 0x606E (Velocity Window Time) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x606E = {DEFTYPE_UNSIGNED16, 0x10, (ACCESS_READWRITE|OBJACCESS_NOPDOMAPPING)};
/** \brief Object 0x606E (Velocity Window Time) object name*/
OBJCONST UCHAR OBJMEM aName0x606E[] = "Velocity Window Time";

/** \brief Object 0x606F (Velocity threshold) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x606F = {DEFTYPE_UNSIGNED16, 0x10, (ACCESS_READWRITE|OBJACCESS_NOPDOMAPPING)};
/** \brief Object 0x606F (Velocity threshold) object name*/
OBJCONST UCHAR OBJMEM aName0x606F[] = "Velocity threshold";

/** \brief Object 0x6070 (Velocity threshold time) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6070 = {DEFTYPE_UNSIGNED16, 0x10, (ACCESS_READWRITE|OBJACCESS_NOPDOMAPPING)};
/** \brief Object 0x6070 (Velocity threshold time) object name*/
OBJCONST UCHAR OBJMEM aName0x6070[] = "Velocity threshold time";

/** \brief Object 0x6071 (Target Torque) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6071 = {DEFTYPE_INTEGER16, 0x10, (ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING)};
/** \brief Object 0x6071 (Target Torque) object name*/
OBJCONST UCHAR OBJMEM aName0x6071[] = "Target Torque";

/** \brief Object 0x6072 (Max.Torque) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6072 = {DEFTYPE_UNSIGNED16, 0x10, (ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING)};
/** \brief Object 0x6072 (Max.Torque) object name*/
OBJCONST UCHAR OBJMEM aName0x6072[] = "Max.Torque";

/** \brief Object 0x6074 (Torque Demand Value) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6074 = {DEFTYPE_INTEGER16, 0x10, (ACCESS_READ|OBJACCESS_TXPDOMAPPING)};
/** \brief Object 0x6074 (Torque Demand Value) object name*/
OBJCONST UCHAR OBJMEM aName0x6074[] = "Torque Demand Value";

/** \brief Object 0x6075 (Motor Rated Current) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6075 = {DEFTYPE_UNSIGNED32, 0x20, (ACCESS_READ|OBJACCESS_NOPDOMAPPING)};
/** \brief Object 0x6075 (Motor Rated Current) object name*/
OBJCONST UCHAR OBJMEM aName0x6075[] = "Motor Rated Current";

/** \brief Object 0x6076 (Motor Rated Torque) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6076 = {DEFTYPE_UNSIGNED32, 0x20, (ACCESS_READ|OBJACCESS_NOPDOMAPPING)};
/** \brief Object 0x6076 (Motor Rated Torque) object name*/
OBJCONST UCHAR OBJMEM aName0x6076[] = "Motor Rated Torque";

/** \brief Object 0x6077 (Torque Actual Value) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6077 = {DEFTYPE_INTEGER16, 0x10, (ACCESS_READ|OBJACCESS_TXPDOMAPPING)};
/** \brief Object 0x6077 (Torque Actual Value) object name*/
OBJCONST UCHAR OBJMEM aName0x6077[] = "Torque Actual Value";

/** \brief Object 0x6078 (Current Actual Value) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6078 = {DEFTYPE_INTEGER16, 0x10, (ACCESS_READ|OBJACCESS_TXPDOMAPPING)};
/** \brief Object 0x6078 (Current Actual Value) object name*/
OBJCONST UCHAR OBJMEM aName0x6078[] = "Current Actual Value";

/** \brief Object 0x6079 (DC Link Circuit Actual Value) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6079 = {DEFTYPE_UNSIGNED32, 0x20, (ACCESS_READ|OBJACCESS_TXPDOMAPPING)};
/** \brief Object 0x6079 (Current Actual Value) object name*/
OBJCONST UCHAR OBJMEM aName0x6079[] = "DC Link Circuit Actual Value";

/** \brief Object 0x607A (Target Position) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x607A = {DEFTYPE_INTEGER32, 0x20, (ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING)};
/** \brief Object 0x607A (Target Position) object name*/
OBJCONST UCHAR OBJMEM aName0x607A[] = "Target Position";

/** \brief Object 0x607C (Home Offset) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x607C = {DEFTYPE_INTEGER32, 0x20, (ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING)};
/** \brief Object 0x607C (Home Offset) object name*/
OBJCONST UCHAR OBJMEM aName0x607C[] = "Home Offset";

/** 
 * \brief Object 0x607D (Software Position Limit) entry description
 * 
 * Subindex 0<br>
 * Subindex 1 : Min Position range limit<br>
 * Subindex 2 : Max Position range limit
 */
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x607D[] = {
    {DEFTYPE_UNSIGNED8, 0x08, ACCESS_READ}, 
    {DEFTYPE_INTEGER32, 0x20, ACCESS_READWRITE},
    {DEFTYPE_INTEGER32, 0x20, ACCESS_READWRITE}};
/** \brief Object 0x607D (Software Position Limit) object and entry names*/
OBJCONST UCHAR OBJMEM aName0x607D[] = "Software Position Limit\000Min position limit\000Max position limit\000\377";

/** \brief Object 0x607E (Polarity) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x607E = {DEFTYPE_UNSIGNED8, 0x8, ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING};
/** \brief Object 0x607E (Polarity) object name*/
OBJCONST UCHAR OBJMEM aName0x607E[] = "Polarity";

/** \brief Object 0x607F (Max.Profile Velocity) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x607F = {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING};
/** \brief Object 0x607F (Max.Profile Velocity) object name*/
OBJCONST UCHAR OBJMEM aName0x607F[] = "Max.Profile Velocity";

/** \brief Object 0x6080 (Max motor speed) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6080 = {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING};
/** \brief Object 0x6080 (Max motor speed) object name*/
OBJCONST UCHAR OBJMEM aName0x6080[] = "Max Motor Speed";

/** \brief Object 0x6081 (Profile Velocity) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6081 = {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING};
/** \brief Object 0x6081 (Profile Velocity) object name*/
OBJCONST UCHAR OBJMEM aName0x6081[] = "Profile Velocity";

/** \brief Object 0x6083 (Profile Acceleration) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6083 = {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE};
/** \brief Object 0x6083 (Profile Acceleration) object name*/
OBJCONST UCHAR OBJMEM aName0x6083[] = "Profile Acceleration";

/** \brief Object 0x6084 (Profile Deceleration) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6084 = {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE};
/** \brief Object 0x6084 (Profile Deceleration) object name*/
OBJCONST UCHAR OBJMEM aName0x6084[] = "Profile Deceleration";

/** \brief Object 0x6085 (Quickstop Declaration) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6085 = {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE};
/** \brief Object 0x6085 (Quickstop Declaration) object name*/
OBJCONST UCHAR OBJMEM aName0x6085[] = "Quickstop Deceleration";

/** \brief Object 0x6087 (Torque Slope) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6087 = {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING};
/** \brief Object 0x6087 (Torque Slope) object name*/
OBJCONST UCHAR OBJMEM aName0x6087[] = "Torque Slope";

/** \brief Object 0x6091 ( Gear Ratio) entry description
* Subindex 0<br>
* Subindex 1 : Motor revolutions
* Subindex 2 : Shaft revolutions
* */
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x6091[] = {
    {DEFTYPE_UNSIGNED8, 0x08, ACCESS_READ},
    {DEFTYPE_UNSIGNED32, 0x20,  ACCESS_READWRITE},
    {DEFTYPE_UNSIGNED32, 0x20,  ACCESS_READWRITE}};
/** \brief Object 0x6091 ( Gear Ratio) object and entry names*/
OBJCONST UCHAR OBJMEM aName0x6091[] = "Gear Ratio\000Motor revolutions\000Shaft revolutions\000\377";


///** \brief Object 0x6093 ( Position Factor) entry description
//* Subindex 0<br>
//* Subindex 1 : numerator
//* Subindex 2 : divisor
//* */
//OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x6093[] = {
//    {DEFTYPE_UNSIGNED8, 0x08, ACCESS_READ},
//    {DEFTYPE_UNSIGNED32, 0x020, ACCESS_READWRITE}};
///** \brief Object 0x6093 ( Position Factor) object and entry names*/
//OBJCONST UCHAR OBJMEM aName0x6093[] = "Position Factor\000Numerator\000Divisor\000\377";
//
///** \brief Object 0x6094 ( Velocity Factor) entry description
//*
//* Subindex 0<br>
//* Subindex 1 : numerator
//* Subindex 2 : divisor
//* */
//OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x6094[] = {
//    {DEFTYPE_UNSIGNED8, 0x08, ACCESS_READ},
//    {DEFTYPE_UNSIGNED32, 0x020, ACCESS_READWRITE}};
///** \brief Object 0x6094 ( Velocity Factor) object and entry names*/
//OBJCONST UCHAR OBJMEM aName0x6094[] = "Velocity Factor\000Numerator\000Divisor\000\377";
//
///** \brief Object 0x6097 ( Acceleration Factor) entry description
//* Subindex 0<br>
//* Subindex 1 : numerator
//* Subindex 2 : divisor
// * */
//OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x6097[] = {
//    {DEFTYPE_UNSIGNED8, 0x08, ACCESS_READ},
//    {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE}};
///** \brief Object 0x6097 ( Acceleration Factor) object and entry names*/
//OBJCONST UCHAR OBJMEM aName0x6097[] = "Acceleration Factor\000Numerator\000Divisor\000\377";

/** \brief Object 0x6098 (Homing Method) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6098 = {DEFTYPE_INTEGER8, 0x8, ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING};
/** \brief Object 0x6098 (Homing Method) object name*/
OBJCONST UCHAR OBJMEM aName0x6098[] = "Homing Method";

/**
 * \brief Object 0x6099 (Homing Speeds) entry description
 *
 * Subindex 0<br>
 * Subindex 1 : Speed during search for switch<br>
 * Subindex 2 : Speed during search for zero
 */
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x6099[] = {
    {DEFTYPE_UNSIGNED8, 0x08, ACCESS_READ},
    {DEFTYPE_UNSIGNED32, 0x20,  ACCESS_READWRITE},
    {DEFTYPE_UNSIGNED32, 0x20,  ACCESS_READWRITE}};
/** \brief Object 0x6099 (Homing Speeds) object and entry names*/
OBJCONST UCHAR OBJMEM aName0x6099[] = "Homing Speeds\000Speed during search for switch\000Speed during search for zero\000\377";

/** \brief Object 0x609A (Homing Acceleration) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x609A = {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READWRITE};
/** \brief Object 0x609A (Homing Acceleration) object name*/
OBJCONST UCHAR OBJMEM aName0x609A[] = "Homing Acceleration";

/** \brief Object 0x60B0 (Position Offset) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x60B0 = {DEFTYPE_INTEGER32, 0x20, ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING};
/** \brief Object 0x60B0 (Position Offset) object name*/
OBJCONST UCHAR OBJMEM aName0x60B0[] = "Position Offset";

/** \brief Object 0x60B1 (Velocity Offset) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x60B1 = {DEFTYPE_INTEGER32, 0x20, ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING};
/** \brief Object 0x60B1 (Velocity Offset) object name*/
OBJCONST UCHAR OBJMEM aName0x60B1[] = "Velocity Offset";

/** \brief Object 0x60B2 (Torque Offset) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x60B2 = {DEFTYPE_INTEGER16, 0x10, ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING};
/** \brief Object 0x60B2 (Torque Offset) object name*/
OBJCONST UCHAR OBJMEM aName0x60B2[] = "Torque Offset";

///** \brief Object 0x60B8 (Touch Probe Function) entry description*/
//OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x60B8 = {DEFTYPE_INTEGER16, 0x10, ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING};
///** \brief Object 0x60B8 (Touch Probe Function) object name*/
//OBJCONST UCHAR OBJMEM aName0x60B8[] = "Touch Probe Function";
//
///** \brief Object 0x60B9 (Touch Probe Status) entry description*/
//OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x60B9 = {DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ|OBJACCESS_TXPDOMAPPING};
///** \brief Object 0x60B9 (Touch Probe Status) object name*/
//OBJCONST UCHAR OBJMEM aName0x60B9[] = "Touch Probe Status";
//
///** \brief Object 0x60BA (Touch Probe 1 Positive Edge Position Value) entry description*/
//OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x60BA = {DEFTYPE_INTEGER32, 0x20, ACCESS_READ|OBJACCESS_TXPDOMAPPING};
///** \brief Object 0x60BA (Touch Probe 1 Positive Edge Position Value) object name*/
//OBJCONST UCHAR OBJMEM aName0x60BA[] = "Touch Probe 1 Positive Edge Position Value";
//
///** \brief Object 0x60BB (Touch Probe 1 Negative Edge Position Value) entry description*/
//OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x60BB = {DEFTYPE_INTEGER32, 0x20, ACCESS_READ|OBJACCESS_TXPDOMAPPING};
///** \brief Object 0x60BB (Touch Probe 1 Negative Edge Position Value) object name*/
//OBJCONST UCHAR OBJMEM aName0x60BB[] = "Touch Probe 1 Negative Edge Position Value";
//
///** \brief Object 0x60BC (Touch Probe 2 Positive Edge Position Value) entry description*/
//OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x60BC = {DEFTYPE_INTEGER32, 0x20, ACCESS_READ|OBJACCESS_TXPDOMAPPING};
///** \brief Object 0x60BC (Touch Probe 2 Positive Edge Position Value) object name*/
//OBJCONST UCHAR OBJMEM aName0x60BC[] = "Touch Probe 2 Positive Edge Position Value";
//
///** \brief Object 0x60BD (Touch Probe 2 Negative Edge Position Value) entry description*/
//OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x60BD = {DEFTYPE_INTEGER32, 0x20, ACCESS_READ|OBJACCESS_TXPDOMAPPING};
///** \brief Object 0x60BD (Touch Probe 2 Negative Edge Position Value) object name*/
//OBJCONST UCHAR OBJMEM aName0x60BD[] = "Touch Probe 2 Negative Edge Position Value";

///** \brief Object 0x60C0 (Interpolation sub mode select) entry description*/
//OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x60C0 = {DEFTYPE_INTEGER16, 0x10, ACCESS_READWRITE|OBJACCESS_TXPDOMAPPING};
///** \brief Object 0x60C0 (Interpolation sub mode select) object name*/
//OBJCONST UCHAR OBJMEM aName0x60C0[] = "Interpolation sub mode select";


/**
 * \brief Object 0x60C1 (Interpolation Data Record) entry description
 *
 * Subindex 0<br>
 * Subindex 1 : Interpolation time period value<br>
 * Subindex 2 : Interpolation time index
 */
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x60C1[] = {
    {DEFTYPE_UNSIGNED8, 0x08, ACCESS_READ},
    {DEFTYPE_INTEGER32, 0x20, ACCESS_READWRITE}};
/** \brief Object 0x60C1 (Interpolation Data Record) object and entry names*/
OBJCONST UCHAR OBJMEM aName0x60C1[] = "Interpolation Data Record\000Interpolation data record\000\377";

/** 
 * \brief Object 0x60C2 (Interpolation Time Period) entry description
 *
 * Subindex 0<br>
 * Subindex 1 : Interpolation time period value<br>
 * Subindex 2 : Interpolation time index
 */
OBJCONST TSDOINFOENTRYDESC    OBJMEM asEntryDesc0x60C2[] = {
    {DEFTYPE_UNSIGNED8, 0x08, ACCESS_READ},
    {DEFTYPE_UNSIGNED8, 0x08, ACCESS_READWRITE},
    {DEFTYPE_INTEGER8,  0x08, ACCESS_READWRITE}};

/** \brief Object 0x60C2 (Interpolation Time Period) object and entry names*/
OBJCONST UCHAR OBJMEM aName0x60C2[] = "Interpolation Time Period\000Interpolation time period\000Interpolation time index\000\377";

/** \brief Object 0x60C5 (Max acceleration) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x60C5 = {DEFTYPE_UNSIGNED32, 0x20, (ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING)};
/** \brief Object 0x60C5 (Max acceleration) object name*/
OBJCONST UCHAR OBJMEM aName0x60C5[] = "Max acceleration";

/** \brief Object 0x60C6 (Min deceleration) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x60C6 = {DEFTYPE_UNSIGNED32, 0x20, (ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING)};
/** \brief Object 0x60C6 (Min deceleration) object name*/
OBJCONST UCHAR OBJMEM aName0x60C6[] = "Min deceleration";

/** \brief Object 0x60E0 (Positive Torque Limit Value) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x60E0 = {DEFTYPE_UNSIGNED16, 0x10, (ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING)};
/** \brief Object 0x60E0 (Positive Torque Limit Value) object name*/
OBJCONST UCHAR OBJMEM aName0x60E0[] = "Positive Torque Limit Value";

/** \brief Object 0x60E1 (Negative Torque Limit Value) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x60E1 = {DEFTYPE_UNSIGNED16, 0x10, (ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING)};
/** \brief Object 0x60E1 (Negative Torque Limit Value) object name*/
OBJCONST UCHAR OBJMEM aName0x60E1[] = "Negative Torque Limit Value";

/** \brief Object 0x60F4 (Following Error Actual Value) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x60F4 = {DEFTYPE_INTEGER32, 0x20, (ACCESS_READ|OBJACCESS_TXPDOMAPPING)};
/** \brief Object 0x60F4 (Following Error Actual Value) object name*/
OBJCONST UCHAR OBJMEM aName0x60F4[] = "Following Error Actual Value";

/** \brief Object 0x60FC (Position Demand Internal Value) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x60FC = {DEFTYPE_INTEGER32, 0x20, (ACCESS_READ|OBJACCESS_TXPDOMAPPING)};
/** \brief Object 0x60FC (Position Demand Internal Value) object name*/
OBJCONST UCHAR OBJMEM aName0x60FC[] = "Position Demand Internal Value";

/** \brief Object 0x60FD (Digital Input) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x60FD = {DEFTYPE_UNSIGNED32, 0x20, (ACCESS_READ|OBJACCESS_TXPDOMAPPING)};
/** \brief Object 0x60FD (Digital Input) object name*/
OBJCONST UCHAR OBJMEM aName0x60FD[] = "Digital Input";

/** \brief Object 0x60FE (Digital Output) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x60FE = {DEFTYPE_UNSIGNED32, 0x20, (ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING)};
/** \brief Object 0x60FE (Digital Output) object name*/
OBJCONST UCHAR OBJMEM aName0x60FE[] = "Digital Output";

/** \brief Object 0x60FF (Target Velocity) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x60FF = {DEFTYPE_INTEGER32, 0x20, (ACCESS_READWRITE|OBJACCESS_RXPDOMAPPING)};
/** \brief Object 0x60FF (Target Velocity) object name*/
OBJCONST UCHAR OBJMEM aName0x60FF[] = "Target Velocity";

/** \brief Object 0x6502 (Supported Drive Modes) entry description*/
OBJCONST TSDOINFOENTRYDESC    OBJMEM sEntryDesc0x6502 = {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ};
/** \brief Object 0x6502 (Supported Drive Modes) object name*/
OBJCONST UCHAR OBJMEM aName0x6502[] = "Supported Drive Modes";
/** @}*/

#endif  //#if SERVOTYPE == SERVO_ETHERCAT

#endif  //#ifdef _CiA402_

#ifdef _CiA402_
    #define PROTO
#else
    #define PROTO extern
#endif

#if SERVOTYPE !=SERVO_CANOPEN
PROTO  TCiA402Axis           LocalAxes;
#endif

#if SERVOTYPE == SERVO_ETHERCAT
PROTO  CiA402Objects  const  DefCiA402ObjectValues;
PROTO  CiA402Objects  const  DefCiA402ObjectValuesFull;

PROTO  Obj_Attribute         ObjAttr[];

void Cia402_FirstPwoerUp_Init_M(void);
void Cia402_FirstPwoerUp_Init_O(void);
PROTO void CiA402_Init(void);
PROTO void CiA402_rst(void);
PROTO void CiA402_Application(TCiA402Axis *pCiA402Axis);

PROTO void   APPL_AckErrorInd(UINT16 stateTrans);
PROTO UINT16 APPL_StartMailboxHandler(void);
PROTO UINT16 APPL_StopMailboxHandler(void);
PROTO UINT16 APPL_StartInputHandler(UINT16 *pIntMask);
PROTO UINT16 APPL_StopInputHandler(void);
PROTO UINT16 APPL_StartOutputHandler(void);
PROTO UINT16 APPL_StopOutputHandler(void);

PROTO UINT16 APPL_GenerateMapping(UINT16 *pInputSize,UINT16 *pOutputSize);
PROTO void APPL_InputMapping(UINT16* pData);
PROTO void APPL_OutputMapping(UINT16* pData);
PROTO UINT8 CiA402_TransitionAction(INT16 Characteristic);
PROTO UINT8 CiA402_HaltStopAction(INT16 Characteristic);
PROTO UINT8 CiA402_Stop(TCiA402Axis *pCiA402Axis);
PROTO void CiA402_StateMachine(void);
PROTO void CiA402_AppFeedbackHanle(void);
PROTO Obj_Attribute * Obj_GetObjAttriute(UINT16 index,UINT8 subindex);
#endif        //#if SERVOTYPE == SERVO_ETHERCAT

#undef PROTO  //#define PROTO extern

#endif        //_CIA402_H_
/** @}*/
