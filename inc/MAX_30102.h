#ifndef MAX_30102_H
#define MAX_30102_H

#include <stdint.h>

#define MAX30102_I2C_ADDR   0x57 
#define MAX30102_INT_PIN    16

/* 7-bit I2C address (datasheet: AE/AF as 8-bit) */
#define MAX30102_I2C_ADDR        0x57   /* 0xAE >> 1 [file:1] */
#define MAX30102_FIFO_LEN        0x20   //FIFO len

/* Register addresses (datasheet register map) [file:1] */
#define MAX30102_REG_INT_STATUS1       0x00
#define MAX30102_REG_INT_STATUS2       0x01
#define MAX30102_REG_INT_ENABLE1       0x02
#define MAX30102_REG_INT_ENABLE2       0x03
#define MAX30102_REG_FIFO_WR_PTR       0x04
#define MAX30102_REG_OVF_COUNTER       0x05
#define MAX30102_REG_FIFO_RD_PTR       0x06
#define MAX30102_REG_FIFO_DATA         0x07
#define MAX30102_REG_FIFO_CONFIG       0x08
#define MAX30102_REG_MODE_CONFIG       0x09
#define MAX30102_REG_SPO2_CONFIG       0x0A
#define MAX30102_REG_LED1_PA           0x0C
#define MAX30102_REG_LED2_PA           0x0D
#define MAX30102_REG_MULTI_LED_CTRL1   0x11
#define MAX30102_REG_MULTI_LED_CTRL2   0x12
#define MAX30102_REG_TEMP_INT          0x1F
#define MAX30102_REG_TEMP_FRAC         0x20
#define MAX30102_REG_TEMP_CONFIG       0x21
#define MAX30102_REG_REV_ID            0xFE
#define MAX30102_REG_PART_ID           0xFF

#define LEDCURRENT_MA(x)                ((uint8_t)(x/0.2))

/* Expected PART_ID value (0x15) [file:1] */
#define MAX30102_PART_ID_EXPECTED      0x15

/* MODE_CONFIG bits [file:1] */
#define MAX30102_MODE_SHDN             0x80
#define MAX30102_MODE_RESET            0x40
#define MAX30102_MODE_MASK             0x07

/* SPO2_CONFIG fields [file:1] */
#define MAX30102_SPO2_ADC_RGE_MASK     0x60
#define MAX30102_SPO2_SR_MASK          0x1C
#define MAX30102_SPO2_PW_MASK          0x03

/* Useful SPO2 settings: 18-bit, 100sps, full scale 16�A [file:1] */
#define MAX30102_SPO2_ADC_RGE_16384NA  (0x03 << 5)
#define MAX30102_SPO2_SR_100_SPS       (0x01 << 2)
#define MAX30102_LED_PW_411US_18BIT    0x03

/* FIFO_CONFIG bits [file:1] */
#define MAX30102_FIFO_A_FULL_MASK      0x0F
#define MAX30102_FIFO_ROLLOVER_EN      0x10
#define MAX30102_FIFO_SMP_AVE_MASK     0xE0

/* MAX30102 Register Configuration Enums */

/* FIFO_CONFIG (0x08) - SMP_AVE[7:5] [file:1] */
typedef enum {
    MAX30102_SMP_AVE_1    = 0x00,  /* 1 sample (no averaging) */
    MAX30102_SMP_AVE_2    = 0x01,  /* 2 samples averaged */
    MAX30102_SMP_AVE_4    = 0x02,  /* 4 samples averaged */
    MAX30102_SMP_AVE_8    = 0x03,  /* 8 samples averaged */
    MAX30102_SMP_AVE_16   = 0x04,  /* 16 samples averaged */
    MAX30102_SMP_AVE_32   = 0x05,  /* 32 samples averaged */
    MAX30102_SMP_AVE_32A  = 0x06,  /* 32 samples averaged */
    MAX30102_SMP_AVE_32B  = 0x07,  /* 32 samples averaged */
} max30102_sample_ave_t;

/* SPO2_CONFIG (0x0A) - SPO2_SR[4:2] [file:1] */
typedef enum {
    MAX30102_SR_50_SPS    = 0x00,  /* 50 samples/sec */
    MAX30102_SR_100_SPS   = 0x01,  /* 100 samples/sec */
    MAX30102_SR_200_SPS   = 0x02,  /* 200 samples/sec */
    MAX30102_SR_400_SPS   = 0x03,  /* 400 samples/sec */
    MAX30102_SR_800_SPS   = 0x04,  /* 800 samples/sec */
    MAX30102_SR_1000_SPS  = 0x05,  /* 1000 samples/sec */
    MAX30102_SR_1600_SPS  = 0x06,  /* 1600 samples/sec */
    MAX30102_SR_3200_SPS  = 0x07,  /* 3200 samples/sec */
} max30102_sample_rate_t;

/* SPO2_CONFIG (0x0A) - LED_PW[1:0] [file:1] */
typedef enum {
    MAX30102_PW_69US_15BIT  = 0x00,  /* 69�s, 15-bit ADC */
    MAX30102_PW_118US_16BIT = 0x01,  /* 118�s, 16-bit ADC */
    MAX30102_PW_215US_17BIT = 0x02,  /* 215�s, 17-bit ADC */
    MAX30102_PW_411US_18BIT = 0x03   /* 411�s, 18-bit ADC */
} max30102_pulse_width_t;

/* SPO2_CONFIG (0x0A) - SPO2_ADC_RGE[6:5] [file:1] */
typedef enum {
    MAX30102_RGE_2048NA  = 0x00,  /* 2�A full scale */
    MAX30102_RGE_4096NA  = 0x01,  /* 4�A full scale */
    MAX30102_RGE_8192NA  = 0x02,  /* 8�A full scale */
    MAX30102_RGE_16384NA = 0x03   /* 16�A full scale */
} max30102_adc_range_t;

/* Multi-LED Control SLOTx[2:0] [file:1] */
typedef enum {
    MAX30102_SLOT_OFF     = 0x00,  /* No LED */
    MAX30102_SLOT_RED     = 0x01,  /* LED1 (Red) */
    MAX30102_SLOT_IR      = 0x02,  /* LED2 (IR) */
    MAX30102_SLOT_OFF2    = 0x03   /* No LED */
} max30102_led_slot_t;

/* MODE_CONFIG (0x09) - MODE[2:0] [file:1] */
typedef enum {
    MAX30102_MODE_HR      = 0x02,  /* Heart Rate (Red only) */
    MAX30102_MODE_SPO2    = 0x03   /* SpO2 (Red + IR) */
} max30102_mode_t;

// Temperature Enable
#define MAX30102_TEMP_CONFIG_EN      0x01

// Bit masks from datasheet
#define MAX30102_INT_A_FULL_EN       0x80  // bit 7 INT_EN1
#define MAX30102_INT_PPG_RDY_EN      0x40  // bit 6 INT_EN1
#define MAX30102_INT_ALC_OVF_EN      0x20  // bit 5 INT_EN1
#define MAX30102_INT_PWR_RDY_EN      0x01  // bit 0 INT_EN1
#define MAX30102_INT_DIE_TEMP_RDY_EN 0x02  // bit 2 INT_EN2

// Bitmask for interrupt sources in our driver
#define MAX_INT_SRC_A_FULL        (1u << 0)
#define MAX_INT_SRC_PPG_RDY       (1u << 1)
#define MAX_INT_SRC_ALC_OVF       (1u << 2)
#define MAX_INT_SRC_PWR_RDY       (1u << 3)
#define MAX_INT_SRC_DIE_TEMP_RDY  (1u << 4)

typedef struct {
    uint8_t sample_avg;
    uint8_t fifo_rollover_en;
    uint8_t fifo_full_trigger;
    uint8_t sample_rate;
    uint8_t pulse_width;
    uint8_t adc_range;
    uint8_t slot1;
    uint8_t slot2;
    uint8_t slot3;
    uint8_t slot4;
    uint8_t redled_current;
    uint8_t irled_current;
    uint8_t mode;
} max30102_config_t;

/* Low-level hooks you already implemented (provided elsewhere) */
int max30102_reg_write(uint8_t reg, const uint8_t *data, uint16_t len);
int max30102_reg_read(uint8_t reg, uint8_t *data, uint16_t len);

/* High-level API */
int max30102_check_id(uint8_t *part_id);
int max30102_reset(void);
int max30102_init_spo2_default(max30102_config_t config);

int max30102_enable_temperature();
int max30102_get_temperature(float *ptrTemp);
int max30102_read_temperature(float *temp_c);

int max30102_fifo_clear(void);
int max30102_fifo_read_sample(uint32_t *red, uint32_t *ir);

int max30102_reg_write(uint8_t reg, const uint8_t *data, uint16_t len);
int max30102_reg_read(uint8_t reg, uint8_t *data, uint16_t len);


// static int max30102_set_bits(uint8_t reg, uint8_t mask);

// Interrupt Functions
int max30102_enable_a_full_int(void);
int max30102_enable_ppg_rdy_int(void);
int max30102_enable_alc_ovf_int(void);
int max30102_enable_die_temp_rdy_int(void);
int max30102_enable_all_interrupts(void);
int max30102_disable_all_interrupts(void);
int max30102_print_interrupt_source(void);
int max30102_get_interrupt_source(uint32_t *src_mask);

int max30102_read_fifoRaw(uint8_t *sampleBuffer,
        int max_samples);

int max30102_read_all_fifo_if_ppg_ready(uint32_t int_src_mask,
                                        uint32_t *red_buf,
                                        uint32_t *ir_buf,
                                        int max_samples);
int max30102_test_fifo_reread_once(void);
/* Utility function to convert sample rate enum to string */
const char* max30102_sample_rate_to_string(max30102_sample_rate_t rate);
#endif /* MAX30102_H */
