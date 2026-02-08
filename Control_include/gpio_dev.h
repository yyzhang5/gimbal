#ifndef __GPIO_DEV_H__
#define __GPIO_DEV_H__


#include "main.h"
#include "stdint.h"




extern void set_led(uint8_t led_index, uint8_t status);
extern void toggle_led(uint8_t led_index);
extern uint8_t read_key(uint8_t key_index);





#endif

