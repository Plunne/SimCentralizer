#include "shifterG27.h"

uint8_t getGear(ShifterG27_t *s)
{
	uint8_t gear = 0;

	 if (s->reverse) {
        // Reverse gear
        gear = 9;

    } else {
        // Y = 0
        if((s->yAxis > YAXISLOW) && (s->yAxis < YAXISHIGH)) {
            gear = 0; // Neutral
        // Y != 0
        } else {
            // X-
            if (s->xAxis < XAXISLOW) {
                if (s->yAxis > YAXISHIGH)       gear = 1;   // X- Y+
                else if (s->yAxis < YAXISLOW)   gear = 2;   // X- Y-
            }
            // X=
            else if ((s->xAxis > XAXISLOW) && (s->xAxis < XAXISHIGH)) {
                if (s->yAxis > YAXISHIGH)       gear = 3;   // X= Y+
                else if (s->yAxis < YAXISLOW)   gear = 4;   // X= Y-
            }
            // X+
            else if ((s->xAxis > XAXISHIGH)) {
                if (s->yAxis > YAXISHIGH)       gear = 5;   // X+ Y+
                else if (s->yAxis < YAXISLOW)   gear = 6;   // X+ Y-
            }
        }
    }

	return gear;
}
