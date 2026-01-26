#ifndef __MEM_MAP__
#define __MEM_MAP__
//Mem Map and the Videocore Memory Functions

#include "mem_map.h"
#include "gpio_utilities.h"
#include "i2c_utilities.h"

// Virtual memory pointers to acceess GPIO, DMA and PWM from user space
MEM_MAP gpio_regs, i2c_regs;

// Get uncached memory
void get_uncached_mem(MEM_MAP *mp, int size) {
    if (!map_uncached_mem(mp, size))
        fail("Error: can't allocate uncached memory\n");
}

// Create a FIFO (named pipe)
int create_fifo(char *fname) {
    int ok = 0;

    umask(0);
    if (mkfifo(fname, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH) < 0 && errno != EEXIST)
        printf("Can't open FIFO '%s'\n", fname);
    else
        ok = 1;
    return (ok);
}

// Open a FIFO for writing, return 0 if there is no reader
int open_fifo_write(char *fname) {
    int f = open(fname, O_WRONLY | O_NONBLOCK);

    return (f == -1 ? 0 : f);
}

// Write to FIFO, return 0 if no reader
int write_fifo(int fd, void *data, int dlen) {
    struct pollfd pollfd = {fd, POLLOUT, 0};

    poll(&pollfd, 1, 0);
    if (pollfd.revents & POLLOUT && !(pollfd.revents & POLLERR))
        return (fd ? write(fd, data, dlen) : 0);
    return (0);
}

// Return the free space in FIFO
uint32_t fifo_freespace(int fd) {
    return (fcntl(fd, F_GETPIPE_SZ));
}

// Remove the FIFO
void destroy_fifo(char *fname, int fd) {
    if (fd > 0)
        close(fd);
    unlink(fname);
}

// Map GPIO, DMA and SPI registers into virtual mem (user space)
// If any of these fail, program will be terminated
void map_devices(void) {
    printf("GPIO ");
    map_periph(&gpio_regs, (void *)GPIO_BASE, PAGE_SIZE);
    printf("I2C ");
    map_periph(&i2c_regs, (void *)I2C0_BASE, PAGE_SIZE);
}

// Use mmap to obtain virtual address, given physical
void *map_periph(MEM_MAP *mp, void *base, int size) {
    mp->phys = (void *)((long )base + (long )PHYS_REG_BASE) ;
    mp->size = PAGE_ROUNDUP(size);
    // mp->bus = (uint8_t *)base + (uint8_t *)BUS_REG_BASE;
    mp->bus = (void *)((long )base + (long )BUS_REG_BASE) ;
    mp->virt = map_segment(mp->phys, mp->size);
    printf("Peripheral PHY %p Bus %p Virtual %p Size %d\n", mp->phys, mp->bus, mp->virt, mp->size);
    return(mp->virt);
}

// Allocate uncached memory, get bus & phys addresses
void *map_uncached_mem(MEM_MAP *mp, int size) {
    void *ret;
    mp->size = PAGE_ROUNDUP(size);
    mp->fd = open_mbox();
    ret = (mp->h = alloc_vc_mem(mp->fd, mp->size, DMA_MEM_FLAGS)) > 0 &&
        (mp->bus = lock_vc_mem(mp->fd, mp->h)) != 0 &&
        (mp->virt = map_segment(BUS_PHYS_ADDR(mp->bus), mp->size)) != 0
        ? mp->virt : 0;
    printf("VC mem handle %u, phys %p, virt %p\n", mp->h, mp->bus, mp->virt);
    return(ret);
}

// Free mapped peripheral or memory
void unmap_periph_mem(MEM_MAP *mp) {
    if (mp)
    {
        if (mp->fd)
        {
            unmap_segment(mp->virt, mp->size);
            unlock_vc_mem(mp->fd, mp->h);
            free_vc_mem(mp->fd, mp->h);
            close_mbox(mp->fd);
        }
        else
            unmap_segment(mp->virt, mp->size);
    }
}

// ----- VIDEOCORE MAILBOX -----

// Open mailbox interface, return file descriptor
int open_mbox(void) {
   int fd;

   if ((fd = open("/dev/vcio", 0)) < 0)
       fail("Error: can't open VC mailbox\n");
   return(fd);
}

// Close mailbox interface
void close_mbox(int fd) {
    if (fd >= 0)
        close(fd);
}

// Send message to mailbox, return first response int, 0 if error
uint32_t msg_mbox(int fd, VC_MSG *msgp) {
    uint32_t ret=0, i;

    for (i=msgp->dlen/4; i<=msgp->blen/4; i+=4)
        msgp->uints[i++] = 0;
    msgp->len = (msgp->blen + 6) * 4;
    msgp->req = 0;
    if (ioctl(fd, _IOWR(100, 0, void *), msgp) < 0)
        printf("VC IOCTL failed\n");
    else if ((msgp->req&0x80000000) == 0)
        printf("VC IOCTL error\n");
    else if (msgp->req == 0x80000001)
        printf("VC IOCTL partial error\n");
    else
        ret = msgp->uints[0];
#if DEBUG
    disp_vc_msg(msgp);
#endif
    return(ret);
}

// Allocate memory on PAGE_SIZE boundary, return handle
uint32_t alloc_vc_mem(int fd, uint32_t size, VC_ALLOC_FLAGS flags) {
    VC_MSG msg={.tag=0x3000c, .blen=12, .dlen=12,
        .uints={PAGE_ROUNDUP(size), PAGE_SIZE, flags}};
    return(msg_mbox(fd, &msg));
}

// Lock allocated memory, return bus address
void *lock_vc_mem(int fd, int h) {
    VC_MSG msg={.tag=0x3000d, .blen=4, .dlen=4, .uints={h}};
    return (void *)(h ? (long ) msg_mbox(fd, &msg) : (long )0);
}

// Unlock allocated memory
uint32_t unlock_vc_mem(int fd, int h) {
    VC_MSG msg={.tag=0x3000e, .blen=4, .dlen=4, .uints={h}};
    return(h ? msg_mbox(fd, &msg) : 0);
}

// Free memory
uint32_t free_vc_mem(int fd, int h) {
    VC_MSG msg={.tag=0x3000f, .blen=4, .dlen=4, .uints={h}};
    return(h ? msg_mbox(fd, &msg) : 0);
}

uint32_t set_vc_clock(int fd, int id, uint32_t freq) {
    VC_MSG msg1={.tag=0x38001, .blen=8, .dlen=8, .uints={id, 1}};
    VC_MSG msg2={.tag=0x38002, .blen=12, .dlen=12, .uints={id, freq, 0}};
    msg_mbox(fd, &msg1);
    disp_vc_msg(&msg1);
    msg_mbox(fd, &msg2);
    disp_vc_msg(&msg2);
    return(0);
}

// Display mailbox message
void disp_vc_msg(VC_MSG *msgp) {
    int i;

    printf("VC msg len=%X, req=%X, tag=%X, blen=%x, dlen=%x, data ",
        msgp->len, msgp->req, msgp->tag, msgp->blen, msgp->dlen);
    for (i=0; i<msgp->blen/4; i++)
        printf("%08X ", msgp->uints[i]);
    printf("\n");
}

// ----- VIRTUAL MEMORY -----

// Get virtual memory segment for peripheral regs or physical mem
void *map_segment(void *addr, int size) {
    int fd;
    void *mem;

    size = PAGE_ROUNDUP(size);
    if ((fd = open ("/dev/mem", O_RDWR|O_SYNC|O_CLOEXEC)) < 0)
        fail("Error: can't open /dev/mem, run using sudo\n");
    mem = mmap(0, size, PROT_WRITE|PROT_READ, MAP_SHARED, fd, (long)addr);
    close(fd);
#if DEBUG
    printf("Map %p -> %p\n", (void *)addr, mem);
#endif
    if (mem == MAP_FAILED)
        fail("Error: can't map memory\n");
    return(mem);
}
// Free mapped memory
void unmap_segment(void *mem, int size) {
    if (mem)
        munmap(mem, PAGE_ROUNDUP(size));
}

// Catastrophic failure in initial setup
void fail(char *s) {
    printf(s);
    exit(1);
}
#endif
