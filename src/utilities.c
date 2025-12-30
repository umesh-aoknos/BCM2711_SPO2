#include <stdio.h>
#include <wiringPi.h>
#include "mem_map.h"
#include "i2c_utilities.h"
#include "MAX_30102.h"
#include "utilities.h"
extern MEM_MAP gpio_regs, i2c_regs;
extern MEM_MAP vc_mem;

extern FILE *fpData;
extern char *dataFileName;
extern uint8_t rtcErrorFlag, pingpongIndex;
extern uint8_t pingpongBuffer[2][MAXNUMSAMPLES*BYTESPERSAMPLE];
extern uint32_t pingpongBufferAvailable[2];

void terminate(int sig) {
    switch(sig) {
        case SIGINT:
            printf("CTRL C. User Interrupt\r\n");
            break;
        case READMAX30102_FIFO_ERROR:
            printf("READMAX30102_FIFO_ERROR\r\n");
            break;
        case DONE:
        default:
            printf("DONE\r\n");
            break;
    }

    wiringPiISRStop(MAX30102_INT_PIN);
    max30102_disable_all_interrupts();
    printf("Cleared Interrupts\r\n");
    i2c_end();

    unmap_periph_mem(&vc_mem);

    unmap_periph_mem(&gpio_regs);
    unmap_periph_mem(&i2c_regs);

    printf(" RTC %d. Flush Data from buffer %d. Num Samples %d\r\n", rtcErrorFlag, pingpongIndex, pingpongBufferAvailable[pingpongIndex]);
    uint8_t *sampleBuffer = pingpongBuffer[pingpongIndex ^ 1];
    uint32_t numSamples = pingpongBufferAvailable[pingpongIndex ^ 1];
    fwrite(sampleBuffer, sizeof(uint8_t), numSamples*BYTESPERSAMPLE, fpData);
    if(fpData) {
        fclose(fpData);
        fpData = NULL;
    }

    exit(0);
}

