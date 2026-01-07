#include "MAX_30102.h"
#include "i2c_utilities.h"
#include "utilities.h"
#include "gpio_utilities.h"
#include "mem_map.h"

extern MEM_MAP i2c_regs;

void i2c1_clear_status(void) {
    // Rasp Pi Status Reg config
    // Slave has held the SCL signal
    // Ack Error Clear
    // Transfer Bit for I2C
    // *i2c_s_reg = (I2C_S_CLKT | I2C_S_ERR | I2C_S_DONE);
    *REG32(i2c_regs, I2C_S) |= (I2C_S_CLKT | I2C_S_ERR | I2C_S_DONE);
}

void i2c1_init(void) {
    i2c1_clear_status();

    // write back to control register
    *REG32(i2c_regs, I2C_C) |= (I2C_C_I2CEN | I2C_C_CLEAR);
}

/*
 * Blocking write: send `len` bytes from buf to 7-bit slave `addr`
 * Returns 0 on success, negative on error.
 */
int i2c1_write(uint8_t addr, const uint8_t *buf, uint16_t len) {
    if (len == 0)
        return NOERROR;

    *REG32(i2c_regs, I2C_A) = addr;
    *REG32(i2c_regs, I2C_DLEN) = len;

    // CRITICAL: Clear status + FIFO, **then wait**
    i2c1_clear_status();
    //Clear FIFO
    *REG32(i2c_regs, I2C_C) = I2C_C_CLEAR;

    // *** ADD THIS DELAY (10-50us) ***
    // for (volatile int i = 0; i < 100; i++);  // ~20�s on Pi @ 1GHz
    usleep(10);

    // Now preload FIFO
    // TXD will be set if FIFO Gets full
    // This can happen when len > FIFO length(16 bytes)
    uint32_t status = *REG32(i2c_regs, I2C_S);
    while ((len > 0) && (status & I2C_S_TXD)) {
        *REG8(i2c_regs, I2C_FIFO) = *buf++;
        len--;
        status = *REG32(i2c_regs, I2C_S);
    }

    // Start transfer
    *REG32(i2c_regs, I2C_C) = I2C_C_I2CEN | I2C_C_ST;

    // Loop till done
    // Wait till there is room in FIFO to resume xfer
    int loopCount = I2C_TIMEOUT;
    status = *REG32(i2c_regs, I2C_S);

    //Continue writing to fifo  as more room is made available till remaining = 0 or there's an error
    // while (remaining > 0)
    while (!(status & I2C_S_DONE)) {
        if ((len > 0) && (status & I2C_S_TXD)) {
            *REG8(i2c_regs, I2C_FIFO) = *buf++;
            len--;
            //Reset timeout to wait for next write
            loopCount = I2C_TIMEOUT;
        }

        if (status & (I2C_S_ERR | I2C_S_CLKT)) {
            i2c1_clear_status();
            return I2C_WRITE_ERROR;
        }

        if(loopCount-- < 0) {
            return I2C_WRITE_TIMEOUT;
        }
        status = *REG32(i2c_regs, I2C_S);
    }

    i2c1_clear_status();
    if(len == 0) {
        return NOERROR;
    }
    else {
        return I2C_WRITE_REMNZ_ERROR;
    }
}

int i2c1_read(uint8_t addr, uint8_t *buf, uint16_t len) {
    if (len == 0)
        return NOERROR;

    // program register and data length
    *REG32(i2c_regs, I2C_A) = addr;
    *REG32(i2c_regs, I2C_DLEN) = len;

    // CRITICAL: Clear status + FIFO, **then wait**
    i2c1_clear_status();

    //Clear i2C control
    // start transfer (read)
    *REG32(i2c_regs, I2C_C) = I2C_C_I2CEN | I2C_C_CLEAR | I2C_C_ST | I2C_C_READ;

    // pull data from FIFO while data is available
    int loopCount = I2C_TIMEOUT;
    uint32_t status = *REG32(i2c_regs, I2C_S);
    //i2c xfer complete. Continue emptying fifo till remaining = 0 or there's an error
    //while (!(status & I2C_S_DONE)) {
    while(len > 0) {
        if (status & I2C_S_RXD) {
            *buf++ = *REG8(i2c_regs, I2C_FIFO);
            len--;
            //Reset timeout to wait for next read
            loopCount = I2C_TIMEOUT;
        }
        // error?
        if (status & (I2C_S_ERR | I2C_S_CLKT)) {
            // clear DONE/ERR/CLKT
            i2c1_clear_status();
            return I2C_READ_ERROR;
        }

        if(loopCount-- < 0) {
            return I2C_READ_TIMEOUT;
        }

        status = *REG32(i2c_regs, I2C_S);
    }

    // clear DONE/ERR/CLKT
    i2c1_clear_status();

    if(len == 0) {
        return NOERROR;
    }
    else {
        return I2C_READ_REMNZ_ERROR;
    }

    // return (remaining == 0) ? NOERROR : I2C_READ_REMNZ_ERROR;
}

int i2c_config() {
    /* Set the SPI0 pins to the Alt 0 function to enable SPI0 access on them */

    gpio_mode(I2C0_SDA_PIN, GPIO_ALT0); // CE1
    gpio_mode(I2C0_SCL_PIN, GPIO_ALT0); // CE0
    gpio_set(MAX30102_INT_PIN, GPIO_IN, GPIO_PULLUP);

    return NOERROR; // OK
}


void i2c_end(void) {  
    /* Set all the SPI0 pins back to input */
    gpio_mode(I2C0_SDA_PIN, GPIO_IN); // CE1
    gpio_mode(I2C0_SCL_PIN, GPIO_IN); // CE0
}
