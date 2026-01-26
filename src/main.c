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

pingpong_t pingpongData, pingpongTemp;

FILE *fpTemp = NULL;
float MAX30102_dieTemp = 0.0;

volatile int32_t reasonCode = NOERROR;
volatile int32_t reasonCodeInner = NOERROR;
volatile int32_t reasonCodeISR = NOERROR;

uint8_t allowedIntMask =
        MAX30102_INT_A_FULL_EN
        // |M AX30102_INT_PPG_RDY_EN
        | MAX30102_INT_ALC_OVF_EN
        // | MAX30102_INT_PWR_RDY_EN
        | MAX30102_INT_DIE_TEMP_RDY_EN
        ;

int main(int argc, char **argv) {
    int prevquant_t = -1;
    uint32_t duration = 10;//Seconds
    struct timeval t1;
    // Start timer
    gettimeofday(&t1, NULL);

    initPingPongStruct(&pingpongData, MAXNUMSAMPLES, MAX30102_FIFO_LEN, BYTESPERSAMPLE);
    initPingPongStruct(&pingpongTemp, MAXNUMSAMPLES, 1, 2);
    struct timeval t2;
    double elapsedTime;

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

    printf("Executing Elapsed Time Loop %s %d %3.1f(redLEDCurrent)%d %3.1f(irLEDCurrent)%d fsamp: %s i2c_freq: %s\r\n", argv[0], duration, irLEDCurrent, LEDCURRENT_MA(irLEDCurrent), redLEDCurrent, LEDCURRENT_MA(redLEDCurrent), max30102_sample_rate_to_string(fsamp), BCM2711_i2c_clockfreq_to_string(i2c_freq));

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

    max30102_init();
    i2c_init(i2c_freq);

    printf("MAX30102 ISR attached on GPIO16. Main loop free to run.\n");

    max30102_check_id(&chip_id);

    printf("PART_ID(0xFF): 0x%02X\n", chip_id);  // Expect 0x15

    
    //Version Non (would read i2c_freq)                                                             Struct size 13
    //uint16_t configVersion = 0x8000 |0x0001; //Version 1. Added Version, I2CFreq, float Temp      Struct size 24 (24 + 3 Dummy Byte)
    uint16_t configVersion = 0x8000 |0x0002; //Version 2. Removed dieTemp as we get Temp Log        Struct size 18 (17 + 1 Dummy Byte)
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
    };

    max30102_init_spo2_default(config);
    fwrite(&config, sizeof(max30102_config_t), 1, fpData);

    usleep(10000);

    while (1) {
        // Main processing / networking / logging, etc.
        uint8_t flushFlag= 0;
        outputPingPong(&pingpongData, fpData, flushFlag);
        outputPingPong(&pingpongTemp, fpTemp, flushFlag);

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
        usleep(1000);
    }

    return 0;
}
