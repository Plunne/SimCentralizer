#ifndef SHIFTERG27_H
#define SHIFTERG27_H

#include "stdint.h"

#define XAXISLOW 	160
#define XAXISHIGH 	250
#define YAXISLOW 	160
#define YAXISHIGH 	250

typedef struct
{
	uint8_t button;
	uint8_t xAxis;
	uint8_t yAxis;
	uint8_t reverse;
}ShifterG27_t;

uint8_t getGear(ShifterG27_t *s);

#endif // SHIFTERG27_H

