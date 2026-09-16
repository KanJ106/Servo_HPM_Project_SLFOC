#ifndef __CANOPEN_INTERFACE_H
#define __CANOPEN_INTERFACE_H

#include <stdio.h>
#include "board.h"
#include "CANopen.h"
#include "CO_NMT_Heartbeat.h"
#include <stdio.h>
#include <assert.h>
#include "board.h"
#include "hpm_sysctl_drv.h"
#include "hpm_mchtmr_drv.h"
#include "hpm_canopen.h"
#include "user_config.h"
#include "OD.h"
#include "CO_ODinterface.h"
#include "CO_config.h"
#include "CO_SYNC.h"
#include "Canopenappl.h"

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

uint8_t canopen_init(void);
uint8_t canopen_task(void);
void CanopenProccessFromIsr(void);
extern CO_t *co[1];
#endif