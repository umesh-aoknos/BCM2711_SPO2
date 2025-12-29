#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <wiringPi.h>
#include "MAX_30102.h"
#include "mem_map.h"
#include "i2c_utilities.h"

extern MEM_MAP gpio_regs, i2c_regs;
extern MEM_MAP vc_mem;

extern FILE *fpData;
extern char *dataFileName;

/* Check PART_ID register (0xFF should be 0x15) [file:1] */
int max30102_check_id(uint8_t *part_id) {
    uint8_t id;
    int ret = max30102_reg_read(MAX30102_REG_PART_ID, &id, 1);
    if (ret != 0) {
        return REG_PART_ID_READ_ERROR;
    }

    if (part_id)
        *part_id = id;

    return (id == MAX30102_PART_ID_EXPECTED) ? 0 : PART_ID_ERROR;
}

/* Issue RESET bit in MODE_CONFIG and wait for it to clear [file:1] */
int max30102_reset(void) {
    uint8_t v = MAX30102_MODE_RESET;
    int ret = max30102_reg_write(MAX30102_REG_MODE_CONFIG, &v, 1);
    if (ret != 0) {
        return REG_MODE_CONFIG_WRITE_ERROR;
    }

    /* Poll until RESET bit clears (simple, blocking) */
    do {
        ret = max30102_reg_read(MAX30102_REG_MODE_CONFIG, &v, 1);
        if (ret != 0) {
            return REG_MODE_CONFIG_READ_ERROR;
        }
    } while (v & MAX30102_MODE_RESET);
    usleep(1000);

    /* Making Shutdown 0 so that we resume operation*/
    uint8_t mode = 0x00;  // SHDN=0
    ret = max30102_reg_write(MAX30102_REG_MODE_CONFIG, &mode, 1);
    if (ret != 0) {
        return REG_MODE_CONFIG_WRITE_ERROR;
    }
    usleep(100);

    return 0;
}

/* Configure a sane SpO2 mode default:
 * - FIFO averaged 4 samples, rollover enabled, A_FULL = 0
 * - HR mode, RED led only
 * - ADC range 16�A, 3200 sps, 18-bit @ 411�s
 * - LED currents  0xFE, each 1 bit increment = 0.75ma increment
 */
int max30102_init_spo2_default(max30102_config_t config) {
    int ret;

    /* Reset first */
    ret = max30102_reset();
    if (ret != 0) {
        return ret;
    }

    max30102_enable_all_interrupts();

    /* FIFO: 4x averaging, rollover enabled, interrupt when full */
    uint8_t fifo_config = (config.sample_avg << 5) | (config.fifo_rollover_en << 4) | (config.fifo_full_trigger);
    ret = max30102_reg_write(MAX30102_REG_FIFO_CONFIG, &fifo_config, 1);
    if (ret != 0) {
        return REG_FIFO_CONFIG_WRITE_ERROR;
    }

    /* SpO2: 16�A range, 100sps, 411�s/18-bit pulses */
    uint8_t spo2_config = (config.adc_range << 5) | (config.sample_rate << 2) | (config.pulse_width);
    ret = max30102_reg_write(MAX30102_REG_SPO2_CONFIG, &spo2_config, 1);
    if (ret != 0) {
        return REG_SPO2_CONFIG_WRITE_ERROR;
    }

    /* LED currents: ~35mA each */
    uint8_t led_current = config.redled_current;
    ret = max30102_reg_write(MAX30102_REG_LED1_PA, &led_current, 1); //Red LED
    if (ret != 0) {
        return REG_LED1_PA_WRITE_ERROR;
    }

    led_current = config.irled_current;
    ret = max30102_reg_write(MAX30102_REG_LED2_PA, &led_current, 1);//IR LED
    if (ret != 0) {
        return REG_LED2_PA_WRITE_ERROR;
    }

    /* Multi-LED slots: SLOT1=Red, SLOT2=IR */
    uint8_t slot = (config.slot2 << 4) | config.slot1;
    ret = max30102_reg_write(MAX30102_REG_MULTI_LED_CTRL1, &slot, 1);
    if (ret != 0) {
        return REG_MULTI_LED_CTRL1_WRITE_ERROR;
    }

    slot = (config.slot4 << 4) | config.slot3;
    ret = max30102_reg_write(MAX30102_REG_MULTI_LED_CTRL2, &slot, 1);
    if (ret != 0) {
        return REG_MULTI_LED_CTRL2_WRITE_ERROR;
    }

    /* Clear FIFO pointers */
    ret = max30102_fifo_clear();
    if (ret != 0) {
        return ret;
    }

    /* Enable SpO2 mode */
    uint8_t mode = config.mode;
    ret = max30102_reg_write(MAX30102_REG_MODE_CONFIG, &mode, 1);
    if (ret != 0) {
        return REG_MODE_CONFIG_WRITE_ERROR;
    }

    return 0;
}

/* One-shot die temperature read (float �C) [file:1] */
int max30102_read_temperature(float *temp_c) {
    if (!temp_c)
        return NULL_PTR_ERROR;

    int ret;
    uint8_t v;

    // 1) start conversion
    v = 0x01;
    ret = max30102_reg_write(MAX30102_REG_TEMP_CONFIG, &v, 1);
    printf("TEMP_CFG write ret=%d v=0x%02X\n", ret, v);

    if (ret != 0) {
        return REG_TEMP_CONFIG_WRITE_ERROR;
    }

    // 2) read back TEMP_CONFIG once
    ret = max30102_reg_read(MAX30102_REG_TEMP_CONFIG, &v, 1);
    printf("TEMP_CFG after write: ret=%d v=0x%02X\n", ret, v);
    if (ret != 0) {
        return REG_TEMP_CONFIG_READ_ERROR;
    }

    // 3) poll until TEMP_EN clears (add timeout)
    int tries = 1000;
    while ((v & 0x01) && tries--) {
        ret = max30102_reg_read(MAX30102_REG_TEMP_CONFIG, &v, 1);
        if (ret != 0) {
            return REG_TEMP_CONFIG_READ_ERROR;
        }
    }
    printf("TEMP_CFG after wait: v=0x%02X tries_left=%d\n", v, tries);

    // 4) read integer and fraction
    uint8_t ti = 0, tf = 0;
    ret = max30102_reg_read(MAX30102_REG_TEMP_INT, &ti, 1);
    printf("TINT ret=%d val=0x%02X\n", ret, ti);
    if (ret != 0) {
        return REG_TEMP_INT_READ_ERROR;
    }

    ret = max30102_reg_read(MAX30102_REG_TEMP_FRAC, &tf, 1);
    printf("TFRAC ret=%d val=0x%02X\n", ret, tf);
    if (ret != 0) {
        return REG_TEMP_FRAC_READ_ERROR;
    }

    int8_t ti_s = (int8_t)ti;
    float frac = (tf >> 4) * 0.0625f;
    *temp_c = (float)ti_s + frac;
    printf("temp_c=%f\n", *temp_c);

    return 0;
}



/* Clear FIFO pointers to 0 [file:1] */
int max30102_fifo_clear(void) {
    int ret;
    uint8_t v = 0x00;

    ret = max30102_reg_write(MAX30102_REG_FIFO_WR_PTR, &v, 1);
    if (ret != 0) {
        return REG_FIFO_WR_PTR_WRITE_ERROR;
    }

    ret = max30102_reg_write(MAX30102_REG_OVF_COUNTER, &v, 1);
    if (ret != 0) {
        return REG_OVF_COUNTER_WRITE_ERROR;
    }

    ret = max30102_reg_write(MAX30102_REG_FIFO_RD_PTR, &v, 1);
    if (ret != 0) {
        return REG_FIFO_RD_PTR_WRITE_ERROR;
    }

    return 0;
}

/* Read one SpO2 sample: returns 18-bit left-justified values in 24-bit words
 * (3 bytes per channel, RED then IR) [file:1]
 */
int max30102_fifo_read_sample(uint32_t *red, uint32_t *ir) {
    if (!red || !ir) {
        return NULL_PTR_ERROR;
    }

    uint8_t buf[6];
    int ret = max30102_reg_read(MAX30102_REG_FIFO_DATA, buf, 6);
    if (ret != 0) {
        return REG_FIFO_DATA_READ_ERROR;
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

    return 0;
}

// Write 'len' bytes starting at MAX30102 register 'reg'
int max30102_reg_write(uint8_t reg, const uint8_t *data, uint16_t len) {
    uint8_t buf[1 + len];
    buf[0] = reg;
    for (uint16_t i = 0; i < len; i++)
        buf[1 + i] = data[i];

    // I2C_ADDR_WRITE is 0x57 per datasheet
    return i2c1_write(MAX30102_I2C_ADDR  , buf, 1 + len);
}

// Read 'len' bytes starting at MAX30102 register 'reg'
int max30102_reg_read(uint8_t reg, uint8_t *data, uint16_t len) {
    int ret;

    ret = i2c1_write(MAX30102_I2C_ADDR, &reg, 1);
    if (ret != 0) {
        return ret;
    }

    // Read bytes from that register (read phase)
    ret = i2c1_read(MAX30102_I2C_ADDR, data, len);
    return ret;
}

static int max30102_set_bits(uint8_t reg, uint8_t mask) {
    uint8_t val;

    // Read 1 byte from MAX30102 register 'reg'
    if (max30102_reg_read(reg, &val, 1) < 0)
        return -1;

    val |= mask;

    // Write 1 byte back to MAX30102 register 'reg'
    if (max30102_reg_write(reg, &val, 1) < 0)
        return -1;

    return 0;
}

static int max30102_clr_bits(uint8_t reg, uint8_t mask) {
    uint8_t val;

    // Read 1 byte from MAX30102 register 'reg'
    if (max30102_reg_read(reg, &val, 1) < 0)
        return -1;

    val &= mask;

    // Write 1 byte back to MAX30102 register 'reg'
    if (max30102_reg_write(reg, &val, 1) < 0)
        return -1;

    return 0;
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
int max30102_enable_all_interrupts(void) {
    uint8_t dummy;

    // Clear any pending interrupts first (read status 1 & 2).[file:1]
    if (max30102_reg_read(MAX30102_REG_INT_STATUS1, &dummy, 1) < 0) {
        return REG_INT_STATUS1_READ_ERROR;
    }

    if (max30102_reg_read(MAX30102_REG_INT_STATUS2, &dummy, 1) < 0) {
        return REG_INT_STATUS2_READ_ERROR;
    }

    if (max30102_set_bits(MAX30102_REG_INT_ENABLE1, MAX30102_INT_A_FULL_EN |
                MAX30102_INT_PPG_RDY_EN |
                MAX30102_INT_ALC_OVF_EN) < 0) {
        return REG_INT_ENABLE1_SETBITS_ERROR;
    }

    if (max30102_set_bits(MAX30102_REG_INT_ENABLE2, MAX30102_INT_DIE_TEMP_RDY_EN) < 0) {
        return REG_INT_ENABLE2_SETBITS_ERROR-1;
    }

    return 0;

}

/* Combined: turn off all interrupts */
int max30102_disable_all_interrupts(void) {
    // uint8_t mask = ~(uint8_t )((uint8_t )MAX30102_INT_A_FULL_EN | (uint8_t )MAX30102_INT_PPG_RDY_EN | (uint8_t )MAX30102_INT_ALC_OVF_EN);
    uint8_t mask = (MAX30102_INT_A_FULL_EN | MAX30102_INT_PPG_RDY_EN | MAX30102_INT_ALC_OVF_EN);
    mask = ~mask;
    if (max30102_clr_bits(MAX30102_REG_INT_ENABLE1, mask) < 0) {
        return REG_INT_ENABLE1_CLRBITS_ERROR;
    }

    mask = MAX30102_INT_DIE_TEMP_RDY_EN;
    mask = ~mask;
    if (max30102_clr_bits(MAX30102_REG_INT_ENABLE2, mask) < 0) {
        return REG_INT_ENABLE2_CLRBITS_ERROR;
    }

    return 0;
}

int max30102_print_interrupt_source(void) {
    uint8_t status1 = 0, status2 = 0;
    int ret;

    // Read Interrupt Status 1 and 2 (also clears them).[file:1]
    ret = max30102_reg_read(MAX30102_REG_INT_STATUS1, &status1, 1);
    if (ret != 0) {
        printf("MAX30102: Failed to read INT_STATUS1 (ret=%d)\n", ret);
        return REG_INT_STATUS1_READ_ERROR;
    }

    ret = max30102_reg_read(MAX30102_REG_INT_STATUS2, &status2, 1);
    if (ret != 0) {
        printf("MAX30102: Failed to read INT_STATUS2 (ret=%d)\n", ret);
        return REG_INT_STATUS2_READ_ERROR;
    }

    // No bits set?
    if (status1 == 0 && status2 == 0) {
        printf("MAX30102: No interrupt flags set\n");
        return 0;
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

    return 0;
}

// Returns >=0 bitmask; <0 on I2C error
int max30102_get_interrupt_source(uint32_t *src_mask) {
    uint8_t s1 = 0, s2 = 0;
    int ret;
    uint32_t mask = 0;

    if (!src_mask) {
        return REG_INT_STATUS1_READ_ERROR;
    }

    // Read and clear Interrupt Status 1 and 2.[file:1]
    ret = max30102_reg_read(MAX30102_REG_INT_STATUS1, &s1, 1);
    if (ret != 0) {
        return REG_INT_STATUS1_READ_ERROR;
    }

    ret = max30102_reg_read(MAX30102_REG_INT_STATUS2, &s2, 1);
    if (ret != 0) {
        return REG_INT_STATUS2_READ_ERROR;
    }

    if (s1 & (1u << 7)) mask |= MAX_INT_SRC_A_FULL;        // A_FULL[file:1]
    if (s1 & (1u << 6)) mask |= MAX_INT_SRC_PPG_RDY;       // PPG_RDY[file:1]
    if (s1 & (1u << 5)) mask |= MAX_INT_SRC_ALC_OVF;       // ALC_OVF[file:1]
    if (s1 & (1u << 4)) mask |= MAX_INT_SRC_PWR_RDY;       // PWR_RDY[file:1]
    if (s2 & (1u << 7)) mask |= MAX_INT_SRC_DIE_TEMP_RDY;  // DIE_TEMP_RDY[file:1]

    // printf("MAX30102 interrupt source(s): %d\n", mask);
    *src_mask = mask;
    return 0;
}

uint32_t byte2Sample(uint8_t *buf) {
    uint32_t sample = (buf[0] << 16) | ( buf[1] <<  8) | buf[2];

    sample &= 0x3FFFF;  // 18-bit left-justified.[file:1]

    return sample;
}

int max30102_read_fifoRaw(uint8_t *sampleBuffer, int max_samples) {
    uint8_t wr_ptr = 0, rd_ptr = 0;
    int ret;

    // Read FIFO write and read pointers.[file:1]
    ret = max30102_reg_read(MAX30102_REG_FIFO_WR_PTR, &wr_ptr, 1);
    if (ret != 0) {
        return REG_FIFO_WR_PTR_READ_ERROR;
    }

    ret = max30102_reg_read(MAX30102_REG_FIFO_RD_PTR, &rd_ptr, 1);
    if (ret != 0) {
        return REG_FIFO_RD_PTR_READ_ERROR;
    }

    // Compute number of unread samples (modulo 32 depth).[file:1]
    uint8_t available = (wr_ptr - rd_ptr + MAX30102_FIFO_LEN) % MAX30102_FIFO_LEN;

    if (available == 0) {
        return 0;
    }

    if (available > max_samples) {
        available = max_samples;
    }

    for (int idx = 0; idx < available; idx++) {
        ret = max30102_reg_read(MAX30102_REG_FIFO_DATA, sampleBuffer, 6);
        sampleBuffer += 6;
        if (ret != 0) {
            return REG_FIFO_DATA_READ_ERROR;
        }
    }
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
    if (ret != 0) {
        return REG_FIFO_WR_PTR_READ_ERROR;
    }

    ret = max30102_reg_read(MAX30102_REG_FIFO_RD_PTR, &rd_ptr, 1);
    if (ret != 0) {
        return REG_FIFO_RD_PTR_READ_ERROR;
    }

    // Compute number of unread samples (modulo 32 depth).[file:1]
    uint8_t available = (wr_ptr - rd_ptr + MAX30102_FIFO_LEN) % MAX30102_FIFO_LEN;

    if (available == 0) {
        return 0;
    }

    if (available > max_samples) {
        available = max_samples;
    }

    for (int idx = 0; idx < available; idx++) {
        uint8_t buf[6];
        ret = max30102_reg_read(MAX30102_REG_FIFO_DATA, buf, 6);
        if (ret != 0) {
            return REG_FIFO_DATA_READ_ERROR;
        }
        // After this the read pointer updates or does not update ?
        // Print out the read pointer before and after the read operation
        // Print out the write pointer as well
        // When does the read pointer overflow can we simulate it and read the pointer values

        red_buf[idx] = byte2Sample(buf);
        ir_buf[idx] = byte2Sample(buf + 3);

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
    if (ret != 0) {
        return ret;
    }

    ret = max30102_reg_read(MAX30102_REG_FIFO_RD_PTR, &rd_ptr, 1);
    if (ret != 0) {
        return REG_FIFO_RD_PTR_READ_ERROR;
    }
    uint8_t ptr1 = rd_ptr;

    ret = max30102_reg_read(MAX30102_REG_FIFO_WR_PTR, &wr_ptr, 1);
    if (ret != 0) {
        return REG_FIFO_WR_PTR_READ_ERROR;
    }
    uint8_t ptr_w1 = wr_ptr;

    printf("S0: RED=%u IR=%u, RD_PTR_after_S0=0x%02X, WR_PTR_after_S0 =0x%02X \n", red0, ir0, ptr1,ptr_w1);

    // ---- Read slot 1 (S1) ----
    
    ret = max30102_fifo_read_sample(&red1, &ir1);   // RD_PTR: 1 -> 2
    if (ret != 0) {
        return ret;
    }

    ret = max30102_reg_read(MAX30102_REG_FIFO_RD_PTR, &rd_ptr, 1);
    if (ret != 0) {
        return REG_FIFO_RD_PTR_READ_ERROR;
    }
    uint8_t ptr2 = rd_ptr;
    
    ret = max30102_reg_read(MAX30102_REG_FIFO_WR_PTR, &wr_ptr, 1);
    if (ret != 0) {
        return REG_FIFO_WR_PTR_READ_ERROR;
    }
    uint8_t ptr_w2 = wr_ptr;

    printf("S1: RED=%u IR=%u, RD_PTR_after_S0=0x%02X, WR_PTR_after_S0 =0x%02X \n", red1, ir1, ptr2,ptr_w2);

    // ---- Force RD_PTR back to absolute slot 0 ----
    rd_ptr = ptr1-1;
    ret = max30102_reg_write(MAX30102_REG_FIFO_RD_PTR, &rd_ptr, 1);
    if (ret != 0) {
        return REG_FIFO_RD_PTR_WRITE_ERROR;
    }

    printf("Manually set RD_PTR back to previous slot\n");

    // ---- Reread slot 0 (should be S0 again) ----
    ret = max30102_fifo_read_sample(&red0_r, &ir0_r);   // RD_PTR: 0 -> 1
    if (ret != 0) {
        return ret;
    }

    printf("S0 reread: RED=%u IR=%u\n", red0_r, ir0_r);

    if (red0 == red0_r && ir0 == ir0_r) {
        printf("FIFO 0-1-0 TEST PASSED: S0 == S0_reread\n");
        return 0;
    } else {
        printf("FIFO 0-1-0 TEST FAILED: "
               "S0(RED=%u IR=%u) vs S0_r(RED=%u IR=%u)\n",
               red0, ir0, red0_r, ir0_r);
        return 1;
    }
}

void i2CHandler(int sig) {
    switch(sig) {
        case SIGINT:
            printf("CTRL C. User Interrupt\r\n");
            break;
        default:
            printf("DONE\r\n");
            break;
    }

    wiringPiISRStop(MAX30102_INT_PIN);
    max30102_disable_all_interrupts();
    i2c_end();

    unmap_periph_mem(&vc_mem);

    unmap_periph_mem(&gpio_regs);
    unmap_periph_mem(&i2c_regs);

    if(fpData) {
        fclose(fpData);
        fpData = NULL;
    }

    exit(0);
}
