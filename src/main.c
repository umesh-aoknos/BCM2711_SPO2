#include <stdlib.h>     //exit()
#include <signal.h>     //signal()
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/timeb.h>
#include <time.h>
#include <errno.h>
// #include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <wiringPi.h>
#include "mem_map.h"
#include "i2c_utilities.h"
#include "MAX_30102.h"
#include "utilities.h"

FILE *fpData = NULL;
MEM_MAP vc_mem;

uint8_t pingpongBuffer[2][MAXNUMSAMPLES*BYTESPERSAMPLE];
uint32_t pingpongBufferAvailable[2] = {0, 0};
uint8_t pingpongIndex = PING;
uint8_t rtcErrorFlag = 0;

void max_30102_wiringPiISR() {
    static uint32_t state = 0;
    uint32_t src;

    if (max30102_get_interrupt_source(&src) != 0)
        return;

    // React to other interrupt sources:
    if (src & MAX_INT_SRC_PPG_RDY) {
        // printf("Reacting to PPG Ready Interrupt\r\n");
    }

    if ((src & MAX_INT_SRC_A_FULL) && state) {
        // FIFO almost full maybe increase read rate or log
        // printf("Reacting to FIFO FULL\r\n");

        // Process n samples in red_samples[0..n-1], ir_samples[0..n-1]
        // e.g., push into your DSP / DMA pipeline

        uint32_t numAvailableSamples = pingpongBufferAvailable[pingpongIndex];
        uint8_t *curPtr = NULL;
        if((MAXNUMSAMPLES - numAvailableSamples) > MAX30102_FIFO_LEN*BYTESPERSAMPLE) {
            //stay on current buffer
            curPtr = pingpongBuffer[pingpongIndex] + numAvailableSamples*BYTESPERSAMPLE;
        }
        else {
            //switch buffer
            rtcErrorFlag++;
            if(rtcErrorFlag > 1) {
                printf("RTC Error %d\n", rtcErrorFlag);
            }
            // printf("Current buffer %d has %d samples\r\n", pingpongIndex, pingpongBufferAvailable[pingpongIndex]);
            pingpongIndex ^= 1;
            // printf("Switching to %d buffer\r\n", pingpongIndex);

            //Switch Buffer,reset write Pointer and available sample count
            curPtr = pingpongBuffer[pingpongIndex];
            pingpongBufferAvailable[pingpongIndex] = 0;
        }

        int numSamples = max30102_read_fifoRaw(curPtr, MAX30102_FIFO_LEN);
        if(numSamples < 0) {
            terminate(READMAX30102_FIFO_ERROR);
        }

        //Updated write Pointer and available sample count
        pingpongBufferAvailable[pingpongIndex] += numSamples;
    }

    if (src & MAX_INT_SRC_ALC_OVF) {
        // Ambient light overflow consider adjusting LED current or placement
        printf("Reacting to AMBIENT LIGHT OVERFLOW\r\n");
    }

    if (src & MAX_INT_SRC_DIE_TEMP_RDY) {
        // You can trigger max30102_read_temperature() here if desired.[file:1]
        float temp_c;
        int ret = max30102_get_temperature(&temp_c);
        if(ret >= 0) {
            printf("Die Temp Ready. Temp %3.1f\r\n", temp_c);
            state = 1;
        }
    }
}

int main(int argc, char **argv) {
    /* Mapping the memory for peripherals and
     * mapping the uncached memory to be used*/
    struct timeval t1, t2;
    double elapsedTime;

    // Start timer
    gettimeofday(&t1, NULL);

    map_devices();
    map_uncached_mem(&vc_mem, VC_MEM_SIZE);

    uint8_t chip_id;
    const char *dataDir = "Dataprocessing";
    const char *dataFileNamePrefix = "SPO2Data";
    char dataFileName[MAXSTRLEN];

    snprintf(dataFileName, MAXSTRLEN, "%s//%s%ld.dat", dataDir, dataFileNamePrefix, (long )t1.tv_sec);

    uint32_t duration = 10;//Seconds
    float irLEDCurrent=25, redLEDCurrent=25; //mA
    switch(argc) {
        case 4:
            irLEDCurrent = atof(argv[3]);
            break;
        case 3:
            redLEDCurrent = atof(argv[2]);
            break;
        case 2:
            duration = atoi(argv[1]);
            break;
        default:
            break;
    }

    printf("Executing %s %d %f(redLEDCurrent) %f(irLEDCurrent)\r\n", argv[0], duration, irLEDCurrent, redLEDCurrent);

    if (!fpData) {
        fpData = fopen(dataFileName, "wb");
        printf("Opening File %s. Result %d\r\n", dataFileName, (fpData == NULL));
    }
    
    // Exception handling:ctrl + c
    signal(SIGINT, terminate);

    i2c_config();
    i2c1_init();

    // Attach ISR on GPIO26 falling edge (MAX30102 INT pin)
    // uses BCM numbering of the GPIOs and directly accesses the GPIO registers.
    wiringPiSetupGpio();
    wiringPiISR(MAX30102_INT_PIN, INT_EDGE_FALLING, &max_30102_wiringPiISR);

    printf("MAX30102 ISR attached on GPIO16. Main loop free to run.\n");
    printf("=== MAX30102 Write Test ===\n");

    max30102_check_id(&chip_id);
    printf("PART_ID(0xFF): 0x%02X\n", chip_id);  // Expect 0x15

    max30102_config_t config = {
        MAX30102_SMP_AVE_1,
        1,
        0x0F,
        MAX30102_SR_50_SPS,
        MAX30102_PW_411US_18BIT,
        MAX30102_RGE_8192NA,
        MAX30102_SLOT_RED,
        MAX30102_SLOT_IR,
        MAX30102_SLOT_OFF,
        MAX30102_SLOT_OFF,
        LEDCURRENT_MA(redLEDCurrent), //mA
        LEDCURRENT_MA(irLEDCurrent), //mA
        MAX30102_MODE_SPO2,
    };

    max30102_init_spo2_default(config);

    fwrite(&config, sizeof(max30102_config_t), 1, fpData);
    usleep(10000);

    while (1) {
        // Main processing / networking / logging, etc.
        if(rtcErrorFlag > 0) {
            uint8_t *sampleBuffer = pingpongBuffer[pingpongIndex ^ 1];
            uint32_t numSamples = pingpongBufferAvailable[pingpongIndex ^ 1];
            fwrite(sampleBuffer, sizeof(uint8_t), numSamples*BYTESPERSAMPLE, fpData);
            rtcErrorFlag--;
            // printf("RTC Error %d\n", rtcErrorFlag);
        }
        else {
            usleep(100000);
        }

        gettimeofday(&t2, NULL);
        // Compute elapsed time in milliseconds:
        // Seconds to milliseconds
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        // Microseconds to milliseconds
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        printf("Elapsed time %4.1f/%d secs\r\n", elapsedTime/1000, duration);
        if(elapsedTime > duration*1000) {
            terminate(DONE);
        }
    }

    return 0;
}
