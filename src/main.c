#include <stdlib.h>     //exit()
#include <signal.h>     //signal()
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/timeb.h>
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

FILE *fpData = NULL;
char *dataFileName = "Dataprocessing/SPO2Data.dat";
MEM_MAP vc_mem;

uint32_t duration = 10;//Seconds

void max_30102_wiringPiISR() {
    uint32_t src;

    if (max30102_get_interrupt_source(&src) != 0)
        return;

    // React to other interrupt sources:
    if (src & MAX_INT_SRC_PPG_RDY) {
        // printf("Reacting to PPG Ready Interrupt\r\n");
    }

    if (src & MAX_INT_SRC_A_FULL) {
        // FIFO almost full maybe increase read rate or log
        // printf("Reacting to FIFO FULL\r\n");

        // Process n samples in red_samples[0..n-1], ir_samples[0..n-1]
        // e.g., push into your DSP / DMA pipeline

        static uint8_t sampleBuffer[6*MAX30102_FIFO_LEN];
        int numSamples = max30102_read_fifoRaw(sampleBuffer, MAX30102_FIFO_LEN);
        fwrite(sampleBuffer, sizeof(uint8_t), numSamples*6, fpData);
    }

    if (src & MAX_INT_SRC_ALC_OVF) {
        // Ambient light overflow consider adjusting LED current or placement
        // printf("Reacting to AMBIENT LIGHT OVERFLOW\r\n");
    }

    if (src & MAX_INT_SRC_DIE_TEMP_RDY) {
        // You can trigger max30102_read_temperature() here if desired.[file:1]
        // printf("Reacting to Die Temp Ready\r\n");
    }
}

int main(int argc, char **argv) {
    /* Mapping the memory for peripherals and
     * mapping the uncached memory to be used*/
    map_devices();
    map_uncached_mem(&vc_mem, VC_MEM_SIZE);

    uint8_t chip_id;

    if (!fpData) {
        fpData = fopen(dataFileName, "wb");
        printf("Opening File %s. Result %d\r\n", dataFileName, (fpData == NULL));
    }
    
    // Exception handling:ctrl + c
    signal(SIGINT, i2CHandler);

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
        0x7F,
        0x7F,
        MAX30102_MODE_SPO2,
    };

    max30102_init_spo2_default(config);

    fwrite(&config, sizeof(max30102_config_t), 1, fpData);
    usleep(10000);

    uint32_t loopCount = 0;
    while (1) {
        // Main processing / networking / logging, etc.
        sleep(1);
        printf("Elapsed time %d/%d secs\r\n", loopCount, duration);
        if(loopCount++ > duration) {
            i2CHandler(DONE);
        }
    }

    return 0;
}
