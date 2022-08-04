#include "motherboard.h"

uint8_t getButtonbox(uint8_t *buffer)
{
    uint8_t button = 0;
	uint8_t dec = buffer[1] - '0';
	uint8_t num = buffer[2] - '0';

	if (dec == 1) num += 10;
	
	button += num;

    return button;
}
