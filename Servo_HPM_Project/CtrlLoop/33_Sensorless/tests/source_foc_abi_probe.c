/* Compile for target only. Symbol sizes are compared with the retained
 * Drive_GlobalVariableDefs.o; this detects struct-size ABI mismatches. */
#include "Drive.h"
char abi_DrvCoeff[sizeof(DrvCoeff)];
char abi_AdRead[sizeof(AdRead)];
char abi_Clark[sizeof(Clark)];
char abi_Park[sizeof(Park)];
char abi_Ipark[sizeof(Ipark)];
char abi_IdRef[sizeof(IdRef)];
char abi_IqRef[sizeof(IqRef)];
char abi_IdPiReg[sizeof(IdPiReg)];
char abi_IqPiReg[sizeof(IqPiReg)];
char abi_Svpwm[sizeof(Svpwm)];
char abi_ToqLoop[sizeof(ToqLoop)];
