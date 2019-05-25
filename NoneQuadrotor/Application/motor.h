#ifndef __MOTOR__H__
#define __MOTOR__H__
#include "sys.h"
#include "stdbool.h"


#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define LOST_VEHICLE_DELAY      10  // called at 10hz so 1 second

typedef struct
{

   uint8_t armed             ;    // 0 if disarmed, 1 if armed
   uint8_t interlock         ;    // 1 if the motor interlock is enabled (i.e. motors run), 0 if disabled (motors don't run)
   uint8_t initialised_ok    ;    // 1 if initialisation was successful


}Motors_flags;

extern Motors_flags motors_flags;


bool motor_armed(void);


#endif


