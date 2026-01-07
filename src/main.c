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
FILE *fpTemp = NULL;
MEM_MAP vc_mem;

uint8_t pingpongDataBuffer[2][MAXNUMSAMPLES*BYTESPERSAMPLE];
uint32_t pingpongDataBufferAvailable[2] = {0, 0};
uint8_t pingpongDataIndex = PING;
uint8_t rtcErrorFlag = 0;

float pingpongTempBuffer[2][MAXNUMSAMPLES];
uint32_t pingpongTempBufferAvailable[2] = {0, 0};
uint8_t pingpongTempIndex = PING;
uint8_t tempErrorFlag = 0;

volatile int32_t reasonCode = NOERROR;
volatile int32_t reasonCodeInner = NOERROR;
volatile int32_t reasonCodeISR = NOERROR;

uint8_t deviceReadyForTempMeasurement = 0;
void max_30102_wiringPiISR() {
    static uint8_t deviceReadyForMeasurement = 0;
    uint32_t src;

    int ret = max30102_get_interrupt_source(&src);
    if (ret != NOERROR) {
        reasonCode = ret;
        reasonCodeISR = MAX30102_GET_INTERRUPT_SRC_ERROR;
        return;
    }

    // React to other interrupt sources:
    if (src & MAX_INT_SRC_PPG_RDY) {
        // printf("Reacting to PPG Ready Interrupt\r\n");
    }

    if ((src & MAX_INT_SRC_A_FULL) && deviceReadyForMeasurement) {
        // FIFO almost full maybe increase read rate or log
        // Process n samples in red_samples[0..n-1], ir_samples[0..n-1]
        // e.g., push into your DSP / DMA pipeline

        uint32_t numAvailableSamples = pingpongDataBufferAvailable[pingpongDataIndex];
        uint8_t *curPtr = NULL;
        if((MAXNUMSAMPLES - numAvailableSamples) > MAX30102_FIFO_LEN*BYTESPERSAMPLE) {
            //stay on current buffer
            curPtr = pingpongDataBuffer[pingpongDataIndex] + numAvailableSamples*BYTESPERSAMPLE;
        }
        else {
            //switch buffer
            rtcErrorFlag++;
            if(rtcErrorFlag > 1) {
                printf("RTC Error %d\n", rtcErrorFlag);
            }
            // printf("Current buffer %d has %d samples\r\n", pingpongDataIndex, pingpongDataBufferAvailable[pingpongDataIndex]);
            pingpongDataIndex ^= 1;
            // printf("Switching to %d buffer\r\n", pingpongDataIndex);

            //Switch Buffer,reset write Pointer and available sample count
            curPtr = pingpongDataBuffer[pingpongDataIndex];
            pingpongDataBufferAvailable[pingpongDataIndex] = 0;
        }

        int numSamples = max30102_read_fifoRaw(curPtr, MAX30102_FIFO_LEN);
        // printf("Reacting to FIFO FULL %d\r\n", numSamples);

        if(numSamples < 0) {
            reasonCode = READMAX30102_FIFO_ERROR;
            reasonCodeISR = READMAX30102_FIFO_ERROR;
            return;
        }

        //Updated write Pointer and available sample count
        pingpongDataBufferAvailable[pingpongDataIndex] += numSamples;
    }

    if (src & MAX_INT_SRC_ALC_OVF) {
        // Ambient light overflow consider adjusting LED current or placement
        // printf("Reacting to AMBIENT LIGHT OVERFLOW\r\n");
    }

    if (src & MAX_INT_SRC_DIE_TEMP_RDY) {
        deviceReadyForMeasurement = 1;
        deviceReadyForTempMeasurement = 1;
        #if 0
        uint32_t numAvailableSamples = pingpongTempBufferAvailable[pingpongTempIndex];
        float *curPtr = NULL;
        if((MAXNUMSAMPLES - numAvailableSamples) > sizeof(float)) {
            //stay on current buffer
            curPtr = pingpongTempBuffer[pingpongTempIndex] + numAvailableSamples;
        }
        else {
            //switch buffer
            tempErrorFlag++;
            if(tempErrorFlag > 1) {
                printf("Temp RTC Error %d\n", tempErrorFlag);
            }
            pingpongTempIndex ^= 1;

            //Switch Buffer,reset write Pointer and available sample count
            curPtr = pingpongTempBuffer[pingpongTempIndex];
            pingpongTempBufferAvailable[pingpongTempIndex] = 0;
        }

        // You can trigger max30102_read_temperature() here if desired.[file:1]
        float temp_c;
        ret = max30102_get_temperature(&temp_c);
        printf("Got Temp Interrupt and Temp Data %d,%f\r\n", ret, temp_c);
        if(ret == NOERROR) {
            *curPtr = temp_c;
            //Updated write Pointer and available sample count
            pingpongTempBufferAvailable[pingpongTempIndex] += 1;
        }
        else {
            reasonCode = ret;;
            reasonCodeISR = ret;;
            return;
        }
        #else
        float temp_c;
        ret = max30102_get_temperature(&temp_c);
        printf("Got Temp Interrupt and Temp Data %d,%3.1f\r\n", ret, temp_c);
        #endif
    }
}

int main(int argc, char **argv) {
    uint32_t duration = 10;//Seconds
    struct timeval t1;
    // Start timer
    gettimeofday(&t1, NULL);

#ifdef DEBUG_LOOP_CNT
    uint32_t writeCount = 0;
#else
    struct timeval t2;
    double elapsedTime;
#endif

    /* Mapping the memory for peripherals and
     * mapping the uncached memory to be used*/
    map_devices();
    map_uncached_mem(&vc_mem, VC_MEM_SIZE);

    uint8_t chip_id;
    const char *dataDir = "Dataprocessing";
    const char *dataFileNamePrefix = "SPO2Data";
    const char *tempFileNamePrefix = "SPO2Temp";
    char dataFileName[MAXSTRLEN];
    char tempFileName[MAXSTRLEN];

    snprintf(dataFileName, MAXSTRLEN, "%s//%s%ld.dat", dataDir, dataFileNamePrefix, (long )t1.tv_sec);
    snprintf(tempFileName, MAXSTRLEN, "%s//%s%ld.dat", dataDir, tempFileNamePrefix, (long )t1.tv_sec);

    float irLEDCurrent=25, redLEDCurrent=25; //mA
    switch(argc) {
        case 4:
            irLEDCurrent = atof(argv[3]);
        case 3:
            redLEDCurrent = atof(argv[2]);
        case 2:
            duration = atoi(argv[1]);
            break;
        default:
            break;
    }

#ifdef DEBUG_LOOP_CNT
    printf("Executing Debug Count Loop %s %d %f(redLEDCurrent)%d %f(irLEDCurrent)%d\r\n", argv[0], duration, irLEDCurrent, LEDCURRENT_MA(irLEDCurrent), redLEDCurrent, LEDCURRENT_MA(redLEDCurrent));
#else
    printf("Executing Elapsed Time Loop %s %d %f(redLEDCurrent)%d %f(irLEDCurrent)%d\r\n", argv[0], duration, irLEDCurrent, LEDCURRENT_MA(irLEDCurrent), redLEDCurrent, LEDCURRENT_MA(redLEDCurrent));
#endif

    if (!fpData) {
        fpData = fopen(dataFileName, "wb");
        if(fpData == NULL) {
            printf("Error opening data file %s: %s\r\n", dataFileName, strerror(errno));
            terminate(FILEOPENERROR);
        }
        else {
            printf("Opening File %s. Result %d\r\n", dataFileName, (fpData == NULL));
        }
    }

    if (!fpTemp) {
        fpTemp = fopen(tempFileName, "wb");
        if(fpTemp == NULL) {
            printf("Error opening data file %s: %s\r\n", tempFileName, strerror(errno));
            terminate(FILEOPENERROR);
        }
        else {
            printf("Opening File %s. Result %d\r\n", tempFileName, (fpTemp == NULL));
        }
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
            uint8_t *sampleBuffer = pingpongDataBuffer[pingpongDataIndex ^ 1];
            uint32_t numSamples = pingpongDataBufferAvailable[pingpongDataIndex ^ 1];
            fwrite(sampleBuffer, sizeof(uint8_t), numSamples*BYTESPERSAMPLE, fpData);
            rtcErrorFlag--;
#ifdef DEBUG_LOOP_CNT
            writeCount++;
            printf("Write Count %d/%d secs\r\n", writeCount, duration);
#endif
            printf("RTC Error %d,%d\n", rtcErrorFlag, numSamples);
        }

        if(tempErrorFlag > 0) {
            float *sampleBuffer = pingpongTempBuffer[pingpongTempIndex ^ 1];
            uint32_t numSamples = pingpongTempBufferAvailable[pingpongTempIndex ^ 1];
            fwrite(sampleBuffer, sizeof(uint8_t), numSamples*sizeof(float), fpTemp);
            tempErrorFlag--;
            printf("Got Temp measurement Data %d\r\n", numSamples);
        }

        if(deviceReadyForTempMeasurement) {
            deviceReadyForTempMeasurement = 0;
            //int ret = max30102_enable_temperature();
        }

#ifdef DEBUG_LOOP_CNT
        if(writeCount > duration) {
            reasonCode = DONE;
            terminate(DONE);
        }
#else
        gettimeofday(&t2, NULL);
        // Compute elapsed time in milliseconds:
        // Seconds to milliseconds
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;
        // Microseconds to milliseconds
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        printf("Elapsed time %4.1f/%d secs\r\n", elapsedTime/1000, duration);
        if(reasonCodeISR != NOERROR) {
            terminate(reasonCodeISR);
        }
        if(elapsedTime > duration*1000) {
            reasonCode = DONE;
            terminate(DONE);
        }
#endif
        usleep(100000);
    }

    return 0;
}
