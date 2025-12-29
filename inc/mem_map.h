#ifndef MEM_MAP_H
#define MEM_MAP_H

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <ctype.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
// #include "spi_functions.h"
#include <sys/ioctl.h>
#include <sys/mman.h>


// Raspberry Pi hardware version (0 to 4)
#define RPI_VERSION     4
#define PI_4_REG_BASE   0xFE000000  // Pi 4


#if RPI_VERSION == 0
#define PHYS_REG_BASE   PI_01_REG_BASE
#define CLOCK_HZ        250000000
#define SPI_CLOCK_HZ    400000000
#elif RPI_VERSION == 1
#define PHYS_REG_BASE   PI_01_REG_BASE
#define CLOCK_HZ        250000000
#define SPI_CLOCK_HZ    250000000
#elif RPI_VERSION==2 || RPI_VERSION==3
#define PHYS_REG_BASE   PI_23_REG_BASE
#define CLOCK_HZ        250000000
#define SPI_CLOCK_HZ    250000000
#elif RPI_VERSION==4
#define PHYS_REG_BASE   PI_4_REG_BASE
#define CLOCK_HZ        375000000
#define SPI_CLOCK_HZ    200000000
#endif


// Location of peripheral registers in physical memory
#define PI_01_REG_BASE  0x20000000  // Pi Zero or 1
#define PI_23_REG_BASE  0x3F000000  // Pi 2 or 3
#define PI_4_REG_BASE   0xFE000000  // Pi 4

// If non-zero, print debug information
#define DEBUG           0

// If non-zero, set PWM clock using VideoCore mailbox
#define USE_VC_CLOCK_SET 0

// Get bus address of register
#define REG_BUS_ADDR(m, x)  ((uint32_t)(m.bus)  + (uint32_t)(x))
// Convert uncached memory virtual address to bus address
#define MEM_BUS_ADDR(mp, a) ((uint32_t)a-(uint32_t)mp->virt+(uint32_t)mp->bus)
// Convert bus address to physical address (for mmap)
#define BUS_PHYS_ADDR(a)    ((void *)((long)(a)&~0xC0000000))


// Get virtual 8 and 32-bit pointers to register
#define REG8(m, x)  ((volatile uint8_t *) ((long)(m.virt)+(uint32_t)(x)))
#define REG32(m, x) ((volatile uint32_t *)((long)(m.virt)+(uint32_t)(x)))

// DMA control block macros
#define REG(r, a) REG_BUS_ADDR(r, a)
#define MEM(m, a) MEM_BUS_ADDR(m, a)
#define CBS(n) MEM_BUS_ADDR(mp, &dp->cbs[(n)])

// DMA base address channel 0 to 14
#define DMA_BASE        0x007000
// Clock registers and values
#define CLK_BASE        0x101000
// PWM controller registers
#define PWM_BASE        0x20C000
// Location of peripheral registers in bus memory
#define BUS_REG_BASE    0x7E000000

// Microsecond timer definitions
#define USEC_BASE (PHYS_REG_BASE + 0x3000)

// Get bus address of register
#define REG_BUS_ADDR(m, x)  ((uint32_t)(m.bus)  + (uint32_t)(x))
// Convert uncached memory virtual address to bus address
#define MEM_BUS_ADDR(mp, a) ((uint32_t)a-(uint32_t)mp->virt+(uint32_t)mp->bus)

// Structure for mapped peripheral or memory
typedef struct {
    int fd,         // File descriptor
        h,          // Memory handle
        size;       // Memory size
    void *bus,      // Bus address
        *virt,      // Virtual address
        *phys;      // Physical address
} MEM_MAP;

// Mailbox command/response structure
typedef struct {
    uint32_t len,   // Overall length (bytes)
        req,        // Zero for request, 1<<31 for response
        tag,        // Command number
        blen,       // Buffer length (bytes)
        dlen;       // Data length (bytes)
        uint32_t uints[32-5];   // Data (108 bytes maximum)
} VC_MSG __attribute__ ((aligned (16)));

// Videocore mailbox memory allocation flags, see:
//     https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
typedef enum {
    MEM_FLAG_DISCARDABLE    = 1<<0, // can be resized to 0 at any time. Use for cached data
    MEM_FLAG_NORMAL         = 0<<2, // normal allocating alias. Don't use from ARM
    MEM_FLAG_DIRECT         = 1<<2, // 0xC alias uncached
    MEM_FLAG_COHERENT       = 2<<2, // 0x8 alias. Non-allocating in L2 but coherent
    MEM_FLAG_ZERO           = 1<<4, // initialise buffer to all zeros
    MEM_FLAG_NO_INIT        = 1<<5, // don't initialise (default is initialise to all ones)
    MEM_FLAG_HINT_PERMALOCK = 1<<6, // Likely to be locked for long periods of time
    MEM_FLAG_L1_NONALLOCATING=(MEM_FLAG_DIRECT | MEM_FLAG_COHERENT) // Allocating in L2
} VC_ALLOC_FLAGS;

#define PAGE_SIZE       0x1000
// Round up to nearest page
#define PAGE_ROUNDUP(n) ((n)%PAGE_SIZE==0 ? (n) : ((n)+PAGE_SIZE)&~(PAGE_SIZE-1))


// Buffer definitions
#define MAX_SAMPLES 4096
#define SAMPLE_SIZE 4
#define BUFFER_LEN (MAX_SAMPLES * SAMPLE_SIZE)
#define MAX_BUFFERS 2
#define VC_MEM_SIZE (PAGE_SIZE + (BUFFER_LEN * MAX_BUFFERS))

// fcntl constant to get free FIFO length
#define F_GETPIPE_SZ 1032

#define DMA_MEM_FLAGS (MEM_FLAG_COHERENT|MEM_FLAG_ZERO)

void map_devices(void);
void get_uncached_mem(MEM_MAP *mp, int size);
int create_fifo(char *fname);
int open_fifo_write(char *fname);
int write_fifo(int fd, void *data, int dlen);
uint32_t fifo_freespace(int fd);
void destroy_fifo(char *fname, int fd);
void *map_periph(MEM_MAP *mp, void *phys, int size);
void *map_uncached_mem(MEM_MAP *mp, int size);
void unmap_periph_mem(MEM_MAP *mp);
int open_mbox(void);
void close_mbox(int fd);
uint32_t msg_mbox(int fd, VC_MSG *msgp);
uint32_t alloc_vc_mem(int fd, uint32_t size, VC_ALLOC_FLAGS flags);
void *lock_vc_mem(int fd, int h);
uint32_t unlock_vc_mem(int fd, int h);
uint32_t free_vc_mem(int fd, int h);
uint32_t set_vc_clock(int fd, int id, uint32_t freq);
void disp_vc_msg(VC_MSG *msgp);
void *map_segment(void *addr, int size);
void unmap_segment(void *mem, int size);
void fail(char *s);

#endif
