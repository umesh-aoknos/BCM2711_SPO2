#include <stdio.h>
#include <unistd.h>
#include "gpio_utilities.h"
#include "mem_map.h"
#include <pthread.h>

#define GPIO_DEBUG

extern MEM_MAP gpio_regs;

// Set input or output with pullups
void gpio_set(int pin, int mode, int pull) {
    gpio_mode(pin, mode);
    gpio_pull(pin, pull);
}

// Set I/P pullup or pulldown
void gpio_pull(int pin, int pull) {
    volatile uint32_t *reg = REG32(gpio_regs, GPIO_GPPUD) + (pin >> 4);
    *reg = pull << (pin % 16)*2;
}

// Set input or output
void gpio_mode(int pin, int mode) {
    volatile uint32_t *reg = REG32(gpio_regs, GPIO_MODE0) + pin / 10, shift = (pin % 10) * 3;
    *reg = (*reg & ~(7 << shift)) | (mode << shift);
}

// Set an O/P pin
void gpio_out(int pin, int val) {
    volatile uint32_t *reg = REG32(gpio_regs, val ? GPIO_SET0 : GPIO_CLR0) + pin/32;
    *reg = 1 << (pin % 32);
}

// Get an I/P pin value
uint8_t gpio_in(int pin) {
    volatile uint32_t *reg = REG32(gpio_regs, GPIO_LEV0) + pin/32;
    return (((*reg) >> (pin % 32)) & 1);
}

