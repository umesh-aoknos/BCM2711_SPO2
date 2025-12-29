#ifndef __I2C_UTILITIES__
#define __I2C_UTILITIES__

#include <stdint.h>

// I2C reg base and offset
#define I2C0_BASE        0x804000

// ---------- Pins (BSC1 on GPIO2/3, ALT0) ----------
#define I2C0_SDA_PIN   2
#define I2C0_SCL_PIN   3

// If your driver expects the 8‑bit address as-is (no shifting)
#define I2C_ADDR_WRITE  0xAE
#define I2C_ADDR_READ   0xAF



//------------ Reg defs MLX30102 ----------------------- 
#define MLX_30102_CHIP_ID_REG     0xFF


// ---------- BSC1 register offsets ----------
#define I2C_C          0x00   // Control
#define I2C_S          0x04   // Status
#define I2C_DLEN       0x08   // Data length
#define I2C_A          0x0C   // Slave address
#define I2C_FIFO       0x10   // Data FIFO

// ---------- Control register bits ----------
#define I2C_C_I2CEN    (1 << 15)  // I2C enable
#define I2C_C_ST       (1 << 7)   // Start transfer
#define I2C_C_CLEAR    (1 << 4)   // Clear FIFO
#define I2C_C_READ     (1 << 0)   // 1 = read transfer

// ---------- Status register bits ----------
#define I2C_S_CLKT     (1 << 9)   // Clock stretch timeout
#define I2C_S_ERR      (1 << 8)   // ACK error
#define I2C_S_RXD      (1 << 5)   // FIFO has data to read
#define I2C_S_TXD      (1 << 4)   // FIFO can accept data
#define I2C_S_DONE     (1 << 1)   // Transfer done
#define I2C_S_TA       (1 << 0)   // Transfer active

// ---------- External mapping for BSC1 ----------

// ---------- API ----------
void i2c1_clear_status(void);
void i2c1_init(void);

int  i2c1_write(uint8_t addr, const uint8_t *buf, uint16_t len);
int  i2c1_read (uint8_t addr, uint8_t *buf, uint16_t len);

int  i2c_config(void);   // configure GPIO pins to ALT0
void i2c_end(void);     // restore GPIO pins to input
int max30102_read_chip_id(uint8_t *chip_id);
void i2c1_add_mapper(void);
#endif // __I2C_UTILITIES__
