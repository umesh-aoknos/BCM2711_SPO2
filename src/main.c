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

uint8_t pingpongDataBuffer[2][MAXNUMSAMPLES*BYTESPERSAMPLE];
uint32_t pingpongDataBufferAvailable[2] = {0, 0};
uint8_t pingpongDataIndex = PING;
uint8_t rtcErrorFlag = 0;

#ifdef TEMPLOG
FILE *fpTemp = NULL;
#endif
float MAX30102_dieTemp = 0.0;

volatile int32_t reasonCode = NOERROR;
volatile int32_t reasonCodeInner = NOERROR;
volatile int32_t reasonCodeISR = NOERROR;

uint8_t deviceReadyForTempMeasurement = 0;
//Note ISR Cannot terminate. Needs to return with Code for main loop to process and terminate
void max_30102_wiringPiISR() {
    //Don't set deviceReadyForMeasurement to 1 without updating fwrite for max30102_config_t
    static uint8_t deviceReadyForMeasurement = 0;
    uint32_t src;

    int ret = max30102_get_interrupt_source(&src);
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
        ret = max30102_disable_interrupts(MAX30102_INT_A_FULL_EN);
        if(ret != NOERROR) {
            reasonCodeISR = REG_ENABLE_ALL_INTERRUPT_ERROR;
            reasonCode = ret;
            return;
        }
        // FIFO almost full maybe increase read rate or log
        // Process n samples in red_samples[0..n-1], ir_samples[0..n-1]
        // e.g., push into your DSP / DMA pipeline

        uint32_t numAvailableSamples = pingpongDataBufferAvailable[pingpongDataIndex];
        uint8_t *curPtr = NULL;
        if((MAXNUMSAMPLES - numAvailableSamples) > MAX30102_FIFO_LEN) {
            //stay on current buffer
            curPtr = pingpongDataBuffer[pingpongDataIndex] + numAvailableSamples*BYTESPERSAMPLE;
        }
        else {
            //switch buffer
            ret = max30102_get_temperature(&MAX30102_dieTemp);
            if(ret != NOERROR) {
                reasonCodeISR = REG_TEMP_INT_READ_ERROR;
                reasonCode = ret;
                return;
            }
            ret = max30102_enable_temperature();
            if(ret != NOERROR) {
                reasonCodeISR = REG_TEMP_CONFIG_WRITE_ERROR;
                reasonCode = ret;
                return;
            }
            rtcErrorFlag++;
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
        ret = max30102_enable_interrupts(MAX30102_INT_A_FULL_EN);
        if(ret != NOERROR) {
            reasonCodeISR = REG_ENABLE_ALL_INTERRUPT_ERROR;
            reasonCode = ret;
            return;
        }
    }

    if (src & MAX30102_INT_ALC_OVF_EN) {
        // Ambient light overflow consider adjusting LED current or placement
        // printf("Reacting to AMBIENT LIGHT OVERFLOW\r\n");
    }

    if (src & MAX30102_INT_DIE_TEMP_RDY_EN) {
        deviceReadyForMeasurement = 1;
        deviceReadyForTempMeasurement = 0;
        ret = max30102_disable_interrupts(MAX30102_INT_DIE_TEMP_RDY_EN);
        if(ret != NOERROR) {
            reasonCodeISR = REG_ENABLE_ALL_INTERRUPT_ERROR;
            reasonCode = ret;
            return;
        }
        // ret = max30102_get_temperature(&MAX30102_dieTemp);
        // printf("Got Temp Interrupt and Temp Data %d,%3.1f\r\n", ret, MAX30102_dieTemp);
    }

    if (src & MAX30102_INT_PWR_RDY_EN) {
        printf("Reacting to Power Ready Interrupt\r\n");
    }
}

int main(int argc, char **argv) {
    int prevquant_t = -1;
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
    max30102_sample_rate_t fsamp = MAX30102_SR_50_SPS;
    BCM2711_i2c_clockfreq_t i2c_freq = I2C_100KHz;
    
    char *endptr;//strtol, strtof error handling
    switch(argc) {
        case 6:
            i2c_freq = intToI2CFreq(strtol(argv[5], &endptr, 10));
            if(errno) {
                printf("Error reading i2c_freq: %s\r\n", strerror(errno));
                terminate(ARGCREADERROR);
            }
        case 5:
            fsamp = intToSampleRate(strtol(argv[4], &endptr, 10));
            if(errno) {
                printf("Error reading fsamp: %s\r\n", strerror(errno));
                terminate(ARGCREADERROR);
            }
        case 4:
            irLEDCurrent = strtof(argv[3], &endptr);
            if(errno) {
                printf("Error reading irLEDCurrent: %s\r\n", strerror(errno));
                terminate(ARGCREADERROR);
            }
        case 3:
            redLEDCurrent = strtof(argv[2], &endptr);
            if(errno) {
                printf("Error reading redLEDCurrent: %s\r\n", strerror(errno));
                terminate(ARGCREADERROR);
            }
        case 2:
            duration = strtol(argv[1], &endptr, 10);
            if(errno) {
                printf("Error reading duration: %s\r\n", strerror(errno));
                terminate(ARGCREADERROR);
            }
            break;
        default:
            printf("Using Default Params\r\n");
            break;
    }

#ifdef DEBUG_LOOP_CNT
    printf("Executing Debug Count Loop %s %d %3.1f(redLEDCurrent)%d %3.1f(irLEDCurrent)%d fsamp: %s i2c_freq: %s\r\n", argv[0], duration, irLEDCurrent, LEDCURRENT_MA(irLEDCurrent), redLEDCurrent, LEDCURRENT_MA(redLEDCurrent), max30102_sample_rate_to_string(fsamp), BCM2711_i2c_clockfreq_to_string(i2c_freq));
#else
    printf("Executing Elapsed Time Loop %s %d %3.1f(redLEDCurrent)%d %3.1f(irLEDCurrent)%d fsamp: %s i2c_freq: %s\r\n", argv[0], duration, irLEDCurrent, LEDCURRENT_MA(irLEDCurrent), redLEDCurrent, LEDCURRENT_MA(redLEDCurrent), max30102_sample_rate_to_string(fsamp), BCM2711_i2c_clockfreq_to_string(i2c_freq));
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

#ifdef TEMPLOG
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
#endif

    // Exception handling:ctrl + c
    signal(SIGINT, terminate);

    i2c_config(i2c_freq);
    i2c1_init();

    // Attach ISR on GPIO26 falling edge (MAX30102 INT pin)
    // uses BCM numbering of the GPIOs and directly accesses the GPIO registers.
    wiringPiSetupGpio();
    wiringPiISR(MAX30102_INT_PIN, INT_EDGE_FALLING, &max_30102_wiringPiISR);

    printf("MAX30102 ISR attached on GPIO16. Main loop free to run.\n");

    max30102_check_id(&chip_id);

    printf("PART_ID(0xFF): 0x%02X\n", chip_id);  // Expect 0x15

    uint16_t configVersion = 0x8000 |0x0001; //Version 1
    max30102_config_t config = {
        configVersion,
        i2c_freq,
        MAX30102_SMP_AVE_1,
        1,
        0x0F,
        fsamp,
        MAX30102_PW_411US_18BIT,
        MAX30102_RGE_8192NA,
        MAX30102_SLOT_RED,
        MAX30102_SLOT_IR,
        MAX30102_SLOT_OFF,
        MAX30102_SLOT_OFF,
        LEDCURRENT_MA(redLEDCurrent), //mA
        LEDCURRENT_MA(irLEDCurrent), //mA
        MAX30102_MODE_SPO2,
        MAX30102_dieTemp,
    };

    max30102_init_spo2_default(config);
    fwrite(&config, sizeof(max30102_config_t), 1, fpData);

    usleep(10000);

    while (1) {
        // Main processing / networking / logging, etc.
        if(rtcErrorFlag > 0) {
            if(rtcErrorFlag > 1) {
                printf("RTC Error %d\n", rtcErrorFlag);
                reasonCode = RTC_ERROR;
                terminate(RTC_ERROR);
            }

            uint8_t *sampleBuffer = pingpongDataBuffer[pingpongDataIndex ^ 1];
            uint32_t numSamples = pingpongDataBufferAvailable[pingpongDataIndex ^ 1];
            fwrite(sampleBuffer, sizeof(uint8_t), numSamples*BYTESPERSAMPLE, fpData);
#ifdef TEMPLOG
            fwrite(&MAX30102_dieTemp, sizeof(float), 1, fpTemp);
#endif
            rtcErrorFlag--;
#ifdef DEBUG_LOOP_CNT
            writeCount++;
            printf("Write Count %d/%d secs\r\n", writeCount, duration);
#endif
            printf("RTC Error %d,%d, %3.1f\n", rtcErrorFlag, numSamples, MAX30102_dieTemp);
        }

        if(deviceReadyForTempMeasurement) {
            deviceReadyForTempMeasurement = 0;
            config.dieTemp = MAX30102_dieTemp;
            // fwrite(&config, sizeof(max30102_config_t), 1, fpData);
            //int ret = max30102_enable_temperature();
            printf("Got Die Temp Data %3.1f, size %lu\r\n", MAX30102_dieTemp, sizeof(max30102_config_t));
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
        int curquant_t = (int )(elapsedTime/250);
        // Microseconds to milliseconds
        elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
        if(curquant_t > prevquant_t) {
            printf("Elapsed time %4.1f/%d secs (%d, %d)\r\n", elapsedTime/1000, duration, curquant_t, prevquant_t);
            prevquant_t = curquant_t;
        }
        if(reasonCodeISR != NOERROR) {
            terminate(reasonCodeISR);
        }
        if(elapsedTime > duration*1000) {
            reasonCode = DONE;
            terminate(DONE);
        }
#endif
        usleep(1000);
    }

    return 0;
}
