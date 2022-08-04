#ifndef HID_COMMANDS_H
#define HID_COMMANDS_H

#include "stdint.h"

typedef struct
{
	uint8_t buttons[4];
	uint8_t z;
}usbMapping_t;

void pressButton(usbMapping_t *map, uint8_t n);
void releaseButtons(usbMapping_t *map);
void releaseShifter(usbMapping_t *map);

#endif // HID_COMMANDS_H

