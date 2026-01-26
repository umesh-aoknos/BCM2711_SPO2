#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <wiringPi.h>
#include "MAX_30102.h"
#include "gpio_utilities.h"
#include "i2c_utilities.h"
#include "utilities.h"
extern int32_t reasonCode;
extern int32_t reasonCodeInner;
extern int32_t reasonCodeISR;
extern uint8_t allowedIntMask;
extern pingpong_t pingpongData, pingpongTemp;

/* Check PART_ID register (0xFF should be 0x15) [file:1] */
int max30102_check_id(uint8_t *part_id) {
    uint8_t id;
    int ret = max30102_reg_read(MAX30102_REG_PART_ID, &id, 1);
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(PART_ID_ERROR);
    }

    if (part_id)
        *part_id = id;

    return (id == MAX30102_PART_ID_EXPECTED) ? NOERROR : PART_ID_ERROR;
}

/* Issue RESET bit in MODE_CONFIG and wait for it to clear [file:1] */
int max30102_reset(void) {
    uint8_t regValue;
    int ret;
    ret = max30102_set_bits(MAX30102_REG_MODE_CONFIG, MAX30102_MODE_RESET);
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(REG_MODE_CONFIG_WRITE_ERROR);
    }

    /* Poll until RESET bit clears (simple, blocking) */
    do {
        ret = max30102_reg_read(MAX30102_REG_MODE_CONFIG, &regValue, 1);
        if (ret != NOERROR) {
            reasonCode = ret;
            terminate(REG_MODE_CONFIG_READ_ERROR);
        }
    } while (regValue & MAX30102_MODE_RESET);
    usleep(1000);

    /* Making Shutdown 0 so that we resume operation*/
    ret = max30102_clr_bits(MAX30102_REG_MODE_CONFIG, MAX30102_MODE_SHDN);
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(REG_MODE_CONFIG_WRITE_ERROR);
    }
    usleep(100);

    return NOERROR;
}

/******************************************************************************
function:   max_30102_isr
Info:
    ISR for MAX30102 interrupt following BCM2711_SPO2 nomenclature.
    Note: ISR cannot terminate. Sets reasonCodeISR for main loop to process.
******************************************************************************/
void max_30102_isr(void) {
//Note ISR Cannot terminate. Needs to return with Code for main loop to process and terminate
    //Don't set deviceReadyForMeasurement to 1 without updating fwrite for max30102_config_t
    static uint8_t deviceReadyForMeasurement = 0;
    uint32_t src;

    int ret = max30102_get_interrupt_source(&src, allowedIntMask);
    if (ret != NOERROR) {
        reasonCode = ret;
        reasonCodeISR = MAX30102_GET_INTERRUPT_SRC_ERROR;
        return;
    }

    // React to other interrupt sources:
    if (src & MAX30102_INT_PPG_RDY_EN) {
        // printf("Reacting to PPG Ready Interrupt\r\n");
    }

    if ((src & MAX30102_INT_A_FULL_EN) && deviceReadyForMeasurement) {
        // Process n samples in red_samples[0..n-1], ir_samples[0..n-1]
        // e.g., push into your DSP / DMA pipeline
        
        uint8_t *curPtr = getCurPtr(&pingpongData);
        int numSamples = max30102_read_fifoRaw(curPtr, MAX30102_FIFO_LEN);

        if(numSamples < 0) {
            reasonCode = READMAX30102_FIFO_ERROR;
            reasonCodeISR = READMAX30102_FIFO_ERROR;
            return;
        }

        //Updated write Pointer and available sample count
        updatePingPong(&pingpongData, numSamples);

        uint8_t startTempMeasure = 1;
        ret = updateTempPingPong(startTempMeasure);
        if(ret != NOERROR) {
            reasonCode = ret;
            reasonCodeISR = READMAX30102_FIFO_ERROR;
            return;
        }
    }

    if (src & MAX30102_INT_ALC_OVF_EN) {
        // Ambient light overflow consider adjusting LED current or placement
        // printf("Reacting to AMBIENT LIGHT OVERFLOW\r\n");
    }

    if (src & MAX30102_INT_DIE_TEMP_RDY_EN) {
        allowedIntMask =
            MAX30102_INT_A_FULL_EN
            // |M AX30102_INT_PPG_RDY_EN
            | MAX30102_INT_ALC_OVF_EN
            // | MAX30102_INT_PWR_RDY_EN
            // | MAX30102_INT_DIE_TEMP_RDY_EN
            ;

        deviceReadyForMeasurement = 1;
        uint8_t startTempMeasure = 0;
        ret = updateTempPingPong(startTempMeasure);
        if(ret != NOERROR) {
            reasonCode = ret;
            reasonCodeISR = READMAX30102_FIFO_ERROR;
            return;
        }
         printf("Reacting to TEMP\r\n");
    }

    if (src & MAX30102_INT_PWR_RDY_EN) {
        printf("Reacting to Power Ready Interrupt\r\n");
    }

    ret = max30102_enable_interrupts(allowedIntMask);
}

int max30102_init() {
    wiringPiSetupGpio();
    /* Set the SPI0 pins to the Alt 0 function to enable SPI0 access on them */
    gpio_mode(I2C0_SDA_PIN, GPIO_ALT0); // CE1
    gpio_mode(I2C0_SCL_PIN, GPIO_ALT0); // CE0
    gpio_set(MAX30102_INT_PIN, GPIO_IN, GPIO_PULLUP);
    return NOERROR; // OK
}

void max30102_end(void) {  
    /* Set all the SPI0 pins back to input */
    gpio_mode(I2C0_SDA_PIN, GPIO_IN); // CE1
    gpio_mode(I2C0_SCL_PIN, GPIO_IN); // CE0
}

/* Configure a sane SpO2 mode default:
 * - FIFO averaged 4 samples, rollover enabled, A_FULL = 0
 * - HR mode, RED led only
 * - ADC range 16�A, 3200 sps, 18-bit @ 411�s
 * - LED currents  0xFE, each 1 bit increment = 0.75ma increment
 */
int max30102_init_spo2_default(max30102_config_t config) {
    int ret;
    uint8_t regValue[3] = {0, 0, 0};

    /* Reset first */
    ret = max30102_reset();
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(MAX30102_RESET_ERROR);
    }

    //For some reason calling after FIFO_CONFIG causes Interrupts to not trigger
    wiringPiISR(MAX30102_INT_PIN, INT_EDGE_FALLING, &max_30102_isr);

    /* FIFO: 4x averaging, rollover enabled, interrupt when full */
    regValue[0] = (config.sample_avg << 5) | (config.fifo_rollover_en << 4) | (config.fifo_full_trigger);
    /* Enable SpO2 mode */
    regValue[1] = config.mode;
    /* SpO2: 16�A range, 100sps, 411�s/18-bit pulses */
    regValue[2] = (config.adc_range << 5) | (config.sample_rate << 2) | (config.pulse_width);

    //Write to Consecutive regs FIFOCfg, SpO2 Mde, Spo2Cfg
    ret = max30102_reg_write(MAX30102_REG_FIFO_CONFIG, regValue, 3);
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(REG_FIFO_CONFIG_WRITE_ERROR);
    }

    /* LED currents: ~35mA each */
    regValue[0] = config.redled_current;
    regValue[1] = config.irled_current;
    //Write to Consecutive regs REG_LED1_PA (Red LED) REG_LED2_PA (IR Led)
    ret = max30102_reg_write(MAX30102_REG_LED1_PA, regValue, 2);
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(REG_LED1_PA_WRITE_ERROR);
    }

    /* Multi-LED slots: SLOT1=Red, SLOT2=IR */
    regValue[0] = (config.slot2 << 4) | config.slot1;
    regValue[1] = (config.slot4 << 4) | config.slot3;
    ret = max30102_reg_write(MAX30102_REG_MULTI_LED_CTRL1, regValue, 2);
    if (ret != 0) {
        reasonCode = ret;
        terminate(REG_MULTI_LED_CTRL1_WRITE_ERROR);
    }

    ret =max30102_enable_temperature();
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(REG_TEMP_CONFIG_WRITE_ERROR);
    }

    ret = max30102_enable_interrupts(allowedIntMask);
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(REG_ENABLE_ALL_INTERRUPT_ERROR);
    }
    printf("Enabled Interrupts\r\n");

    return NOERROR;
}

int max30102_enable_temperature() {
    uint8_t regValue = MAX30102_TEMP_CONFIG_EN;
    return max30102_reg_write(MAX30102_REG_TEMP_CONFIG, &regValue, 1);
}

/* One-shot die temperature read (float �C) [file:1] */
int max30102_read_temperature(float *ptrTemp) {
    if (!ptrTemp) {
        reasonCode = NULL_PTR_ERROR;
        terminate(NULL_PTR_ERROR);
    }

    int ret;
    uint8_t regValue;

    // 1) start conversion
    regValue = MAX30102_TEMP_CONFIG_EN;
    ret = max30102_reg_write(MAX30102_REG_TEMP_CONFIG, &regValue, 1);

    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(REG_TEMP_CONFIG_WRITE_ERROR);
    }

    // 2) read back TEMP_CONFIG once
    ret = max30102_reg_read(MAX30102_REG_TEMP_CONFIG, &regValue, 1);
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(REG_TEMP_CONFIG_READ_ERROR);
    }

    // 3) poll until TEMP_EN clears (add timeout)
    int tries = 0;
    while ((regValue & MAX30102_TEMP_CONFIG_EN) && tries++) {
        ret = max30102_reg_read(MAX30102_REG_TEMP_CONFIG, &regValue, 1);
        if (ret != NOERROR) {
            reasonCode = ret;
            terminate(REG_TEMP_CONFIG_READ_ERROR);
        }
    }

    printf("TEMP_CFG after wait: value=0x%02X tries=%d\n", regValue, tries);
    return max30102_get_temperature(ptrTemp);
}

int max30102_get_rawTempData(uint8_t *ptrTemp, uint8_t startTempMeasure) {
    int ret = max30102_reg_read(MAX30102_REG_TEMP_INT, ptrTemp, 2);
    if (ret != NOERROR) {
        reasonCode = ret;
        return REG_TEMP_INT_READ_ERROR;
    }

    if(startTempMeasure) {
        ret =max30102_enable_temperature();
        if (ret != NOERROR) {
            reasonCode = ret;
            return REG_TEMP_CONFIG_WRITE_ERROR;
        }
    }
    return NOERROR;
}

int max30102_get_temperature(float *ptrTemp) {
    uint8_t tif[3] = {0, 0, 0};
    int ret = max30102_reg_read(MAX30102_REG_TEMP_INT, tif, 3);
    if (ret != NOERROR) {
        reasonCode = ret;
        return REG_TEMP_INT_READ_ERROR;
    }

    if(tif[2] == 0) {//Temp EN should be zero, else Temp is invalid
        *ptrTemp = tif[0] + (tif[1] >> 4) * 0.0625f;
    }
    else {
        *ptrTemp = -100.0;
    }
    return NOERROR;
}


/* Clear FIFO pointers to 0 [file:1] */
int max30102_fifo_clear(void) {
    int ret;
    uint8_t regValue[3] = {0x00, 0x00, 0x00};

    //Clear consecutive regs WR_PTR, OVC, RD_PTR
    ret = max30102_reg_write(MAX30102_REG_FIFO_WR_PTR, regValue, 3);
    if (ret != 0) {
        reasonCode = ret;
        terminate(REG_FIFO_WR_PTR_WRITE_ERROR);
    }
    return NOERROR;
}

/* Read one SpO2 sample: returns 18-bit left-justified values in 24-bit words
 * (3 bytes per channel, RED then IR) [file:1]
 */
int max30102_fifo_read_sample(uint32_t *red, uint32_t *ir) {
    if (!red || !ir) {
        reasonCode = NULL_PTR_ERROR;
        terminate(NULL_PTR_ERROR);
    }

    uint8_t buf[BYTESPERSAMPLE];
    int ret = max30102_reg_read(MAX30102_REG_FIFO_DATA, buf, BYTESPERSAMPLE);
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(REG_FIFO_DATA_READ_ERROR);
    }

    /* Combine bytes (MSB first), mask 18 bits [file:1] */
    uint32_t r = ((uint32_t)buf[0] << 16) |
                 ((uint32_t)buf[1] <<  8) |
                 ((uint32_t)buf[2]);
    uint32_t i = ((uint32_t)buf[3] << 16) |
                 ((uint32_t)buf[4] <<  8) |
                 ((uint32_t)buf[5]);

    *red = r & 0x3FFFF;
    *ir  = i & 0x3FFFF;

    return NOERROR;
}

// Write 'len' bytes starting at MAX30102 register 'reg'
int max30102_reg_write(uint8_t reg, const uint8_t *data, uint16_t len) {
    uint8_t buf[1 + len];
    buf[0] = reg;
    for (uint16_t i = 0; i < len; i++)
        buf[1 + i] = data[i];

    // I2C_ADDR_WRITE is 0x57 per datasheet
    return i2c_write(MAX30102_I2C_ADDR  , buf, 1 + len);
}

// Read 'len' bytes starting at MAX30102 register 'reg'
int max30102_reg_read(uint8_t reg, uint8_t *data, uint16_t len) {
    int ret;

    ret = i2c_write(MAX30102_I2C_ADDR, &reg, 1);
    if (ret != NOERROR) {
        return ret;
    }

    // Read bytes from that register (read phase)
    ret = i2c_read(MAX30102_I2C_ADDR, data, len);
    return ret;
}

int max30102_set_bits(uint8_t reg, uint8_t mask) {
    uint8_t val;

    // Read 1 byte from MAX30102 register 'reg'

    int ret = max30102_reg_read(reg, &val, 1);
    if (ret != NOERROR) {
        return(ret);
    }

    val |= mask;

    // Write 1 byte back to MAX30102 register 'reg'
    ret = max30102_reg_write(reg, &val, 1);
    if (ret != NOERROR) {
        return(ret);
    }

    return NOERROR;
}

//Clears bits indicated by mask
int max30102_clr_bits(uint8_t reg, uint8_t mask) {
    uint8_t val;

   int ret = max30102_reg_read(reg, &val, 1);
    if (ret != NOERROR) {
		return(ret);
    }

    //Flip mask bits
    mask = ~mask;
    val &= mask;

   // Write 1 byte back to MAX30102 register 'reg'
    ret = max30102_reg_write(reg, &val, 1);
    if (ret != NOERROR) {
		return(ret);
    }
        
    return NOERROR;
}

/* Singular enable functions */
int max30102_enable_a_full_int(void) {
    return max30102_set_bits(MAX30102_REG_INT_ENABLE1, MAX30102_INT_A_FULL_EN);
}

int max30102_enable_ppg_rdy_int(void) {
    return max30102_set_bits(MAX30102_REG_INT_ENABLE1, MAX30102_INT_PPG_RDY_EN);
}

int max30102_enable_alc_ovf_int(void) {
    return max30102_set_bits(MAX30102_REG_INT_ENABLE1, MAX30102_INT_ALC_OVF_EN);
}

int max30102_enable_die_temp_rdy_int(void) {
    return max30102_set_bits(MAX30102_REG_INT_ENABLE2, MAX30102_INT_DIE_TEMP_RDY_EN);
}

/* Combined: turn on all interrupts */
int max30102_enable_interrupts(uint8_t mask) {
    uint8_t regVal[2];

    // Read current INT to apply mask aptly
    int ret = max30102_reg_read(MAX30102_REG_INT_ENABLE1, regVal, 2);
    if (ret != NOERROR) {
        reasonCode = ret;
        return REG_INT_STATUS1_READ_ERROR;
    }

    regVal[0] = regVal[0] | (mask&MAX30102_INT_STAT_REG1_MASK);
    regVal[1] = regVal[1] | (mask&MAX30102_INT_STAT_REG2_MASK);

    // Write masks to both INT
    ret = max30102_reg_write(MAX30102_REG_INT_ENABLE1, regVal, 2);
    if (ret != NOERROR) {
        reasonCode = ret;
        return REG_INT_ENABLE1_SETBITS_ERROR;
    }

    return NOERROR;
}

/* Combined: turn off all interrupts */
int max30102_disable_interrupts(uint8_t mask) {
    uint8_t regVal[2];
    // Read current INT to apply mask aptly
    int ret = max30102_reg_read(MAX30102_REG_INT_ENABLE1, regVal, 2);
    if (ret != NOERROR) {
        reasonCode = ret;
        return REG_INT_STATUS1_READ_ERROR;
    }
    //Flip mask bits
    regVal[0] = regVal[0] & (~(mask&MAX30102_INT_STAT_REG1_MASK));
    regVal[1] = regVal[1] & (~(mask&MAX30102_INT_STAT_REG2_MASK));

    // Write masks to both INT
    ret = max30102_reg_write(MAX30102_REG_INT_ENABLE1, regVal, 2);
    if (ret != NOERROR) {
        reasonCode = ret;
        return REG_INT_ENABLE1_SETBITS_ERROR;
    }

    return NOERROR;
}

int max30102_print_interrupt_source(void) {
    uint8_t status1 = 0, status2 = 0;
    int ret;

    // Read Interrupt Status 1 and 2 (also clears them).[file:1]
    ret = max30102_reg_read(MAX30102_REG_INT_STATUS1, &status1, 1);
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(REG_INT_STATUS1_READ_ERROR);
    }

    ret = max30102_reg_read(MAX30102_REG_INT_STATUS2, &status2, 1);
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(REG_INT_STATUS2_READ_ERROR);
    }

    // No bits set?
    if (status1 == 0 && status2 == 0) {
        printf("MAX30102: No interrupt flags set\n");
        return NOERROR;
    }

    printf("MAX30102 interrupt source(s):\n");

    if (status1 & (1u << 7))
        printf("  - FIFO almost full (A_FULL)\n");        // 0x00, bit 7[file:1]
    if (status1 & (1u << 6))
        printf("  - New PPG sample ready (PPG_RDY)\n");    // 0x00, bit 6[file:1]
    if (status1 & (1u << 5))
        printf("  - Ambient light cancellation overflow (ALC_OVF)\n"); // 0x00, bit 5[file:1]
    if (status1 & (1u << 4))
        printf("  - Power ready (PWR_RDY)\n");             // 0x00, bit 4[file:1]

    if (status2 & (1u << 7))
        printf("  - Die temperature ready (DIE_TEMP_RDY)\n"); // 0x01, bit 7[file:1]

    return NOERROR;
}

// Returns >=0 bitmask; <0 on I2C error
int max30102_get_interrupt_source(uint32_t *src_mask, uint8_t disableIntMask) {
    int ret;
    uint8_t mask = 0;
    if (!src_mask) {
        reasonCodeInner = NULL_PTR_ERROR;
        reasonCode = NULL_PTR_ERROR;
        return NULL_PTR_ERROR;
    }

    uint8_t status[4] = {0, 0, 0, 0};
    // Read and clear Interrupt Status 1 and 2
    ret = max30102_reg_read(MAX30102_REG_INT_STATUS1, status, 4);
    if (ret != NOERROR) {
        reasonCodeInner = ret;
        reasonCode = ret;
        return REG_INT_STATUS1_READ_ERROR;
    }

    //Note we are still checking individual sources and updating the mask
    //as reserved fields in Status Reg1 and Status Reg2 could corrupt the mask
    if (status[0] & MAX30102_INT_A_FULL_EN) {
        mask |= MAX30102_INT_A_FULL_EN;        // A_FULL[file:1]
    }

    if (status[0] & MAX30102_INT_PPG_RDY_EN) {
        mask |= MAX30102_INT_PPG_RDY_EN;       // PPG_RDY[file:1]
    }

    if (status[0] & MAX30102_INT_ALC_OVF_EN) {
        mask |= MAX30102_INT_ALC_OVF_EN;       // ALC_OVF[file:1]
    }

    if (status[0] & MAX30102_INT_PWR_RDY_EN) {
        mask |= MAX30102_INT_PWR_RDY_EN;       // PWR_RDY[file:1]
    }

    if (status[1] & MAX30102_INT_DIE_TEMP_RDY_EN) {
        mask |= MAX30102_INT_DIE_TEMP_RDY_EN;  // DIE_TEMP_RDY[file:1]
    }
    
    //Flip mask bits
    if(disableIntMask) {
        status[0] = status[2] & (~(disableIntMask&MAX30102_INT_STAT_REG1_MASK));
        status[1] = status[3] & (~(disableIntMask&MAX30102_INT_STAT_REG2_MASK));

        // Write masks to both INT
        ret = max30102_reg_write(MAX30102_REG_INT_ENABLE1, status, 2);
        if (ret != NOERROR) {
            reasonCode = ret;
            return REG_INT_ENABLE1_SETBITS_ERROR;
        }
    }

    *src_mask = mask;
    return NOERROR;
}

uint32_t byte2Sample(uint8_t *buf) {
    uint32_t sample = (buf[0] << 16) | ( buf[1] <<  8) | buf[2];

    sample &= 0x3FFFF;  // 18-bit left-justified.[file:1]

    return sample;
}

int max30102_read_fifoRaw(uint8_t *sampleBuffer, int max_samples) {
    uint8_t arrayReadWritePtrs[3] = {0, 0, 0};//WrPtr, WrOVC, RdPtr
    int ret;

    // Read FIFO write and read pointers.[file:1]
    ret = max30102_reg_read(MAX30102_REG_FIFO_WR_PTR, arrayReadWritePtrs, 3);
    if (ret != NOERROR) {
        reasonCode = ret;
        reasonCodeInner = ret;
        return REG_FIFO_WR_PTR_READ_ERROR;
    }

    // Compute number of unread samples (modulo 32 depth).[file:1]
    int available = (arrayReadWritePtrs[0] - arrayReadWritePtrs[2] + MAX30102_FIFO_LEN) % MAX30102_FIFO_LEN;

    if (available == 0) {
        available = 32;
    }

    if (available > max_samples) {
        available = max_samples;
    }

    ret = max30102_reg_read(MAX30102_REG_FIFO_DATA, sampleBuffer, BYTESPERSAMPLE*available);
    sampleBuffer += (BYTESPERSAMPLE*available);
    if (ret != NOERROR) {
        reasonCodeInner = ret;
        reasonCode = REG_FIFO_DATA_READ_ERROR;
        return REG_FIFO_DATA_READ_ERROR;
    }

    reasonCode = NOERROR;
    reasonCodeInner = NOERROR;
    return available;
}

// Read up to 'max_samples' from FIFO when PPG_RDY interrupt is present.
// Returns: >0 = number of samples read, 0 = no PPG interrupt, <0 = error.
int max30102_read_all_fifo_if_ppg_ready(uint32_t int_src_mask,
                                        uint32_t *red_buf,
                                        uint32_t *ir_buf,
                                        int max_samples) {
    uint8_t wr_ptr = 0, rd_ptr = 0;
    int ret;

    // Read FIFO write and read pointers.[file:1]
    ret = max30102_reg_read(MAX30102_REG_FIFO_WR_PTR, &wr_ptr, 1);
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(REG_FIFO_WR_PTR_READ_ERROR);
    }

    ret = max30102_reg_read(MAX30102_REG_FIFO_RD_PTR, &rd_ptr, 1);
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(REG_FIFO_RD_PTR_READ_ERROR);
    }

    // Compute number of unread samples (modulo 32 depth).[file:1]
    uint8_t available = (wr_ptr - rd_ptr + MAX30102_FIFO_LEN) % MAX30102_FIFO_LEN;

    if (available == 0) {
        return NOERROR;
    }

    if (available > max_samples) {
        available = max_samples;
    }

    for (int idx = 0; idx < available; idx++) {
        uint8_t buf[BYTESPERSAMPLE];
        ret = max30102_reg_read(MAX30102_REG_FIFO_DATA, buf, BYTESPERSAMPLE);
        if (ret != NOERROR) {
            reasonCode = ret;
            terminate(REG_FIFO_DATA_READ_ERROR);
        }
        // After this the read pointer updates or does not update ?
        // Print out the read pointer before and after the read operation
        // Print out the write pointer as well
        // When does the read pointer overflow can we simulate it and read the pointer values

        red_buf[idx] = byte2Sample(buf);
        ir_buf[idx] = byte2Sample(buf + BYTESPERLED);

        // printf(" Red and IR samples[%d](%d , %d), \r\n", idx, red_buf[idx], ir_buf[idx]);
    }

    return available;
}

// Test: read S1, S2, then force RD_PTR back to S1 and reread.
// Returns 0 if reread == S1, >0 if mismatch, <0 on error.
int max30102_test_fifo_reread_once(void) {
    int ret;
    uint32_t red0 = 0, ir0 = 0;
    uint32_t red1 = 0, ir1 = 0;
    uint32_t red0_r = 0, ir0_r = 0;

    uint8_t wr_ptr = 0;
    uint8_t rd_ptr = 0;

    // Optional: start from known FIFO state (all pointers 0).[web:82]
    // max30102_fifo_clear();

    // ---- Read slot 0 (S0) ----
    ret = max30102_fifo_read_sample(&red0, &ir0);   // RD_PTR: 0 -> 1
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(ret);
    }

    ret = max30102_reg_read(MAX30102_REG_FIFO_RD_PTR, &rd_ptr, 1);
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(REG_FIFO_RD_PTR_READ_ERROR);
    }
    uint8_t ptr1 = rd_ptr;

    ret = max30102_reg_read(MAX30102_REG_FIFO_WR_PTR, &wr_ptr, 1);
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(REG_FIFO_WR_PTR_READ_ERROR);
    }
    uint8_t ptr_w1 = wr_ptr;

    printf("S0: RED=%u IR=%u, RD_PTR_after_S0=0x%02X, WR_PTR_after_S0 =0x%02X \n", red0, ir0, ptr1,ptr_w1);

    // ---- Read slot 1 (S1) ----
    
    ret = max30102_fifo_read_sample(&red1, &ir1);   // RD_PTR: 1 -> 2
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(ret);
    }

    ret = max30102_reg_read(MAX30102_REG_FIFO_RD_PTR, &rd_ptr, 1);
    if (ret != 0) {
        reasonCode = ret;
        terminate(REG_FIFO_RD_PTR_READ_ERROR);
    }
    uint8_t ptr2 = rd_ptr;
    
    ret = max30102_reg_read(MAX30102_REG_FIFO_WR_PTR, &wr_ptr, 1);
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(REG_FIFO_WR_PTR_READ_ERROR);
    }
    uint8_t ptr_w2 = wr_ptr;

    printf("S1: RED=%u IR=%u, RD_PTR_after_S0=0x%02X, WR_PTR_after_S0 =0x%02X \n", red1, ir1, ptr2,ptr_w2);

    // ---- Force RD_PTR back to absolute slot 0 ----
    rd_ptr = ptr1-1;
    ret = max30102_reg_write(MAX30102_REG_FIFO_RD_PTR, &rd_ptr, 1);
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(REG_FIFO_RD_PTR_WRITE_ERROR);
    }

    printf("Manually set RD_PTR back to previous slot\n");

    // ---- Reread slot 0 (should be S0 again) ----
    ret = max30102_fifo_read_sample(&red0_r, &ir0_r);   // RD_PTR: 0 -> 1
    if (ret != NOERROR) {
        reasonCode = ret;
        terminate(ret);
    }

    printf("S0 reread: RED=%u IR=%u\n", red0_r, ir0_r);

    if (red0 == red0_r && ir0 == ir0_r) {
        printf("FIFO 0-1-0 TEST PASSED: S0 == S0_reread\n");
        return NOERROR;
    } else {
        printf("FIFO 0-1-0 TEST FAILED: "
               "S0(RED=%u IR=%u) vs S0_r(RED=%u IR=%u)\n",
               red0, ir0, red0_r, ir0_r);
        return -1;
    }
}

/* Convert sample rate enum to string representation */
const char* max30102_sample_rate_to_string(max30102_sample_rate_t rate) {
    switch (rate) {
        case MAX30102_SR_50_SPS:
            return "50 SPS";
        case MAX30102_SR_100_SPS:
            return "100 SPS";
        case MAX30102_SR_200_SPS:
            return "200 SPS";
        case MAX30102_SR_400_SPS:
            return "400 SPS";
        case MAX30102_SR_800_SPS:
            return "800 SPS";
        case MAX30102_SR_1000_SPS:
            return "1000 SPS";
        case MAX30102_SR_1600_SPS:
            return "1600 SPS";
        case MAX30102_SR_3200_SPS:
            return "3200 SPS";
        default:
            return "Unknown Sample Rate";
    }
}
