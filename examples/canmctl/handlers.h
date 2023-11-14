#ifndef __HANDLERS_H__
#define __HANDLERS_H__

#include "canfestival.h"

extern volatile int ready;

void Init(CO_Data* d, UNS32 id);
void Exit(CO_Data* d, UNS32 id);

#endif
