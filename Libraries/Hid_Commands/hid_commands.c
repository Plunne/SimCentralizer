#include "hid_commands.h"

void pressButton(usbMapping_t *map, uint8_t n)
{
	switch(n) {
		/* Buttons 0 */
		case 0 : break;
		/* Buttons 1 to 8 */
		case 1 : map->buttons[0] |= (1 << 0); break;
		case 2 : map->buttons[0] |= (1 << 1); break;
		case 3 : map->buttons[0] |= (1 << 2); break;
		case 4 : map->buttons[0] |= (1 << 3); break;
		case 5 : map->buttons[0] |= (1 << 4); break;
		case 6 : map->buttons[0] |= (1 << 5); break;
		case 7 : map->buttons[0] |= (1 << 6); break;
		case 8 : map->buttons[0] |= (1 << 7); break;
		/* Buttons 9 to 16 */
		case 9 : map->buttons[1] |= (1 << 0); break;
		case 10: map->buttons[1] |= (1 << 1); break;
		case 11: map->buttons[1] |= (1 << 2); break;
		case 12: map->buttons[1] |= (1 << 3); break;
		case 13: map->buttons[1] |= (1 << 4); break;
		case 14: map->buttons[1] |= (1 << 5); break;
		case 15: map->buttons[1] |= (1 << 6); break;
		case 16: map->buttons[1] |= (1 << 7); break;
		/* Buttons 17 to 24 */
		case 17: map->buttons[2] |= (1 << 0); break;
		case 18: map->buttons[2] |= (1 << 1); break;
		case 19: map->buttons[2] |= (1 << 2); break;
		case 20: map->buttons[2] |= (1 << 3); break;
		case 21: map->buttons[2] |= (1 << 4); break;
		case 22: map->buttons[2] |= (1 << 5); break;
		case 23: map->buttons[2] |= (1 << 6); break;
		case 24: map->buttons[2] |= (1 << 7); break;
		/* Buttons 25 to 27 */
		case 25: map->buttons[3] |= (1 << 0); break;
		case 26: map->buttons[3] |= (1 << 1); break;
		case 27: map->buttons[3] |= (1 << 2); break;
		case 28: map->buttons[3] |= (1 << 3); break;
		case 29: map->buttons[3] |= (1 << 4); break;
		case 30: map->buttons[3] |= (1 << 5); break;
		/* If invalid input */
		default: break;
	}
}

void releaseButtons(usbMapping_t *map)
{
	map->buttons[1] &= ~ 0b11111100;
	map->buttons[2] = 0;
	map->buttons[3] = 0;
}

void releaseShifter(usbMapping_t *map)
{
	map->buttons[0] = 0;
	map->buttons[1] &= ~ 0b00000011;
}
