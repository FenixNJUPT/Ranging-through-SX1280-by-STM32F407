#include "timoueter.h"
#include "hw.h"
#define TIMER_GET_CURRENT_TIMESTAMP   ( HAL_GetTick() )

Timoueters_t Timoueter_New( )
{
    return (uint32_t) 0;
}

void Timoueter_SetMs( Timoueters_t *timoueter, const uint32_t duration_ms )
{
    *timoueter = TIMER_GET_CURRENT_TIMESTAMP + duration_ms;
}

bool Timoueter_HasExpired( const Timoueters_t *timoueter )
{
    return TIMER_GET_CURRENT_TIMESTAMP > *timoueter;
}
