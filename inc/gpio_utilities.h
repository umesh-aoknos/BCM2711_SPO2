#ifndef __GPIO_UTILITIES_H__
#define __GPIO_UTILITIES_H__

#include <stdint.h>
#include <pthread.h>
#include "mem_map.h"

// GPIO register definitions
#define GPIO_BASE       0x200000
// GPIO register offsets
#define GPIO_MODE0      0x00
#define GPIO_SET0       0x1C
#define GPIO_CLR0       0x28
#define GPIO_LEV0       0x34
#define GPIO_GPPUD      0x94
#define GPIO_GPPUDCLK0  0x98
#define GPIO_GPREN0     0x40
#define GPIO_GPFEN0     0x48
#define GPIO_GPEDS0     0x50

// GPIO modes
#define GPIO_IN             0
#define GPIO_OUT            1
#define GPIO_ALT0           4
#define GPIO_ALT1           5
#define GPIO_ALT2           6
#define GPIO_ALT3           7
#define GPIO_ALT4           3
#define GPIO_ALT5           2
#define GPIO_NOPULL         0
#define GPIO_PULLDN         1
#define GPIO_PULLUP         2

// ISR edge types
typedef enum {
    GPIO_EDGE_RISING  = 1,
    GPIO_EDGE_FALLING = 2,
    GPIO_EDGE_BOTH    = 3
} gpio_edge_t;


// Basic GPIO
uint8_t gpio_in(int pin);
void gpio_out(int pin, int val);
void gpio_mode(int pin, int mode);
void gpio_pull(int pin, int pull);
void gpio_set(int pin, int mode, int pull);
#endif
