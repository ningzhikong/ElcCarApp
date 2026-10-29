#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"

typedef enum {
    SYS_STATE_STANDBY = 0,       
    SYS_STATE_EYE_CATCH,
    SYS_STATE_RUN_ABCD,    
    SYS_STATE_RUN_ACBD,  
    SYS_STATE_STOP,    
} SystemState_t;



#endif
