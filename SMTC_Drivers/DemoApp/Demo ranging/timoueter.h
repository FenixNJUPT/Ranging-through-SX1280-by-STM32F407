#ifndef __TIMOUETER_H__
#define __TIMOUETER_H__

#include <stdbool.h>
#include <stdint.h>

typedef uint32_t Timoueters_t;

Timoueters_t Timoueter_New( void );
void Timoueter_SetMs( Timoueters_t *timoueter, const uint32_t duration_ms );
bool Timoueter_HasExpired( const Timoueters_t *timoueter );

#endif // __TIMOUETER_H__
