#include <stdio.h>
#include <wiringPi.h>
#include "mem_map.h"
#include "i2c_utilities.h"
#include "MAX_30102.h"
#include "utilities.h"
extern MEM_MAP gpio_regs, i2c_regs;
extern MEM_MAP vc_mem;

extern FILE *fpData;
extern FILE *fpTemp;

extern pingpong_t pingpongData, pingpongTemp;

extern int32_t reasonCode;
extern int32_t reasonCodeISR;
extern int32_t reasonCodeInner;
extern int minReadTimeout;
extern int minWriteTimeout;

BCM2711_i2c_clockfreq_t intToI2CFreq(long intArg) {
    BCM2711_i2c_clockfreq_t i2c_freq = I2C_10KHz;
    if(intArg < 15) {
        i2c_freq = I2C_10KHz;
    }
    else if(intArg < 40) {
        i2c_freq = I2C_25KHz;
    }
    else if(intArg < 75) {
        i2c_freq = I2C_50KHz;
    }
    else if(intArg < 150) {
        i2c_freq = I2C_100KHz;
    }
    else if(intArg < 300){  
        i2c_freq = I2C_200KHz;
    }
    else {
        i2c_freq = I2C_200KHz;
    }
    return i2c_freq;
}

max30102_pulse_width_t intToSPO2ADCResolution(long intArg) {
    max30102_pulse_width_t adcRes;
    if(intArg <= 15) {
        adcRes = MAX30102_PW_69US_15BIT;
    }
    else if(intArg == 16) {
        adcRes = MAX30102_PW_118US_16BIT;
    }
    else if(intArg == 17) {
        adcRes = MAX30102_PW_215US_17BIT;
    }
    else {
        adcRes = MAX30102_PW_411US_18BIT;
    }

    return adcRes;
}

max30102_adc_range_t intToSPO2ADCRange(long intArg) {
    max30102_adc_range_t adcRange;
    printf("Range %ld\r\n", intArg);
    if(intArg < 3072) {
        adcRange = MAX30102_RGE_2048NA;
    }
    else if(intArg < 6144) {
        adcRange = MAX30102_RGE_4096NA;
    }
    else if(intArg < 12288) {
        adcRange = MAX30102_RGE_8192NA;
    }
    else {
        adcRange = MAX30102_RGE_16384NA;
    }
    return adcRange;
}

max30102_sample_rate_t intToSPO2SampleRate(long intArg) {
    max30102_sample_rate_t fsamp;
    if(intArg < 75) {
        fsamp = MAX30102_SR_50_SPS;
    }
    else if(intArg < 150) {
        fsamp = MAX30102_SR_100_SPS;
    }
    else if(intArg < 300) {
        fsamp = MAX30102_SR_200_SPS;
    }
    else if(intArg < 600) {
        fsamp = MAX30102_SR_400_SPS;
    }
    else if(intArg < 900) {
        fsamp = MAX30102_SR_800_SPS;
    }
    else if(intArg < 1300){  
        fsamp = MAX30102_SR_1000_SPS;
    }
    else if(intArg < 2400) {
        fsamp = MAX30102_SR_1600_SPS;
    }
    else {
        fsamp = MAX30102_SR_3200_SPS;
    }
    return fsamp;
}

/* Convert BCM2711_i2c_clockfreq_t enum to string */
const char* BCM2711_i2c_clockfreq_to_string(BCM2711_i2c_clockfreq_t freq) {
    switch (freq) {
        case I2C_400KHz:
            return "400 kHz";
        case I2C_200KHz:
            return "200 kHz";
        case I2C_100KHz:
            return "100 kHz";
        case I2C_50KHz:
            return "50 kHz";
        case I2C_25KHz:
            return "25 kHz";
        case I2C_10KHz:
            return "10 kHz";
        default:
            return "Unknown I2C frequency";
    }
}

const char *getErrStr(int err) {
    const char *msg;
    switch (err) {
        case NOERROR:
            msg = "No error";
            break;
        case DONE:
            msg = "DONE";
            break;
        case I2C_WRITE_ERROR:
            msg = "I2C Write Error";
            break;
        case I2C_READ_ERROR:
            msg = "I2C Write Error";
            break;
        case I2C_WRITE_REMNZ_ERROR:
            msg = "I2C NZ Write Error";
            break;
        case I2C_READ_REMNZ_ERROR:
            msg = "I2C NZ Read Error";
            break;
        case READMAX30102_FIFO_ERROR:
            msg = "MAX30102 FIFO Error";
            break;
        case REG_FIFO_DATA_READ_ERROR:
            msg = "FIFO_DATA read failed";
            break;
        case REG_FIFO_DATA_WRITE_ERROR:
            msg = "FIFO DATA write failed";
            break;
        case REG_PART_ID_READ_ERROR:
            msg = "FIFO DATA read failed";
            break;
        case PART_ID_ERROR:
            msg = "PART ID mismatch";
            break;
        case REG_MODE_CONFIG_WRITE_ERROR:
            msg = "MODE_CONFIG write failed";
            break;
        case REG_MODE_CONFIG_READ_ERROR:
            msg = "MODE_CONFIG read failed";
            break;
        case MAX30102_RESET_ERROR:
            msg = "MAX30102 reset failed";
            break;
        case REG_FIFO_CONFIG_WRITE_ERROR:
            msg = "FIFO_CONFIG write failed";
            break;
        case REG_SPO2_CONFIG_WRITE_ERROR:
            msg = "SPO2_CONFIG write failed";
            break;
        case REG_LED1_PA_WRITE_ERROR:
            msg = "LED1_PA (RED) write failed";
            break;
        case REG_LED2_PA_WRITE_ERROR:
            msg = "LED2_PA (IR) write failed";
            break;
        case REG_MULTI_LED_CTRL1_WRITE_ERROR:
            msg = "MULTI_LED_CTRL1 write failed";
            break;
        case REG_MULTI_LED_CTRL2_WRITE_ERROR:
            msg = "MULTI_LED_CTRL2 write failed";
            break;
        case REG_TEMP_CONFIG_WRITE_ERROR:
            msg = "TEMP_CONFIG write failed";
            break;
        case REG_TEMP_CONFIG_READ_ERROR:
            msg = "TEMP_CONFIG read failed";
            break;
        case REG_TEMP_INT_READ_ERROR:
            msg = "TEMP_INT read failed";
            break;
        case REG_TEMP_FRAC_READ_ERROR:
            msg = "TEMP_FRAC read failed";
            break;
        case NULL_PTR_ERROR:
            msg = "Null pointer passed";
            break;
        case REG_OVF_COUNTER_WRITE_ERROR:
            msg = "OVF_COUNTER write failed";
            break;
        case REG_INT_STATUS1_READ_ERROR:
            msg = "INT_STATUS1 read failed";
            break;
        case REG_INT_STATUS2_READ_ERROR:
            msg = "INT_STATUS2 read failed";
            break;
        case REG_INT_ENABLE1_SETBITS_ERROR:
            msg = "INT_ENABLE1 set-bits failed";
            break;
        case REG_INT_ENABLE2_SETBITS_ERROR:
            msg = "INT_ENABLE2 set-bits failed";
            break;
        case REG_INT_ENABLE1_CLRBITS_ERROR:
            msg = "INT_ENABLE1 clear-bits failed";
            break;
        case REG_INT_ENABLE2_CLRBITS_ERROR:
            msg = "INT_ENABLE2 clear-bits failed";
            break;
        case REG_FIFO_WR_PTR_READ_ERROR:
            msg = "FIFO_RD_PTR write failed";
            break;
        case REG_FIFO_WR_PTR_WRITE_ERROR:
            msg = "FIFO_RD_PTR write failed";
            break;
        case REG_FIFO_RD_PTR_READ_ERROR:
            msg = "FIFO_RD_PTR write failed";
            break;
        case REG_FIFO_RD_PTR_WRITE_ERROR:
            msg = "FIFO_RD_PTR write failed";
            break;
        case FILEOPENERROR:
            msg = "File open error";
            break;
        case REG_ENABLE_ALL_INTERRUPT_ERROR:
            msg = "Enable Interrupts error";
            break;
        case MAX30102_GET_INTERRUPT_SRC_ERROR:
            msg = "Get Interrupt source error";
            break;
        case I2C_READ_TIMEOUT:
            msg = "I2C Read Timeout error";
            break;
        case I2C_WRITE_TIMEOUT:
            msg = "I2C Write Timeout error";
            break;
        case ARGCREADERROR:
            msg = "Input argument read error";
            break;
        default:
            msg = "Unknown error";
            break;
    }
    return msg;
}

void terminate(int err) {
    const char *msgErr = getErrStr(err);
    const char *msgReason = getErrStr(reasonCode);
    const char *msgInner = getErrStr(reasonCodeInner);

    printf("Exiting due to %d(reason:%d, %s. Inner Reason:%d, %s) %s\r\n", err, reasonCode, msgReason, reasonCodeInner, msgInner, msgErr);
    //Disable Interrupts
    //Stop ISR
    wiringPiISRStop(MAX30102_INT_PIN);
    printf("ISR Stopped\r\n");
    if(err != PART_ID_ERROR) {
        uint8_t intMask = 
            MAX30102_INT_A_FULL_EN |
            MAX30102_INT_PPG_RDY_EN |
            MAX30102_INT_ALC_OVF_EN |
            MAX30102_INT_PWR_RDY_EN
            | MAX30102_INT_DIE_TEMP_RDY_EN
            ;

        max30102_disable_interrupts(intMask);
    }
    printf("Disabled Interrupts\r\n");

    //Shut Down MAX 30102
    max30102_set_bits(MAX30102_REG_MODE_CONFIG, MAX30102_MODE_SHDN);

    max30102_end();
    printf("i2C GPIO Stopped\r\n");

    unmap_periph_mem(&vc_mem);
    printf("Unmapped virt mem\r\n");

    unmap_periph_mem(&gpio_regs);
    printf("unmapeed GPIO Regs\r\n");

    unmap_periph_mem(&i2c_regs);
    printf("unmapeed I2C Regs\r\n");

        
    uint8_t flushFlag= 1;
    outputPingPong(&pingpongData, fpData, flushFlag);
    outputPingPong(&pingpongTemp, fpTemp, flushFlag);

    clearPingPong(&pingpongData);
    clearPingPong(&pingpongTemp);
    printf("Data flushed\r\n");

    if(fpData) {
        fclose(fpData);
        fpData = NULL;
    }

    if(fpTemp) {
        fclose(fpTemp);
        fpTemp = NULL;
    }

    printf("pingpong flushed and fpData/fpTemp closed. I2C Max Read Timeout %d. I2C Max Write Timeout %d. Exiting\r\n", I2C_TIMEOUT - minReadTimeout, I2C_TIMEOUT - minWriteTimeout);

    exit(0);
}

int updateTempPingPong(uint8_t startTempMeasure) {
        uint8_t *curTempPtr = getCurPtr(&pingpongTemp);
        int ret = max30102_get_rawTempData(curTempPtr, startTempMeasure);
        if(ret != NOERROR) {
            reasonCode = ret;
            return REG_TEMP_INT_READ_ERROR;
        }

        //Print the first temp
        if(startTempMeasure == 0) {
            float temp = *curTempPtr + ((*(curTempPtr + 1))*0.0625f);
            printf("Got Temp Interrupt and Temp Data %d,%d, %5.2f\r\n", *curTempPtr, *(curTempPtr + 1), temp);
        }

        //Updated write Pointer and available sample count
        uint32_t numSamples = 1;
        updatePingPong(&pingpongTemp, numSamples);

        return NOERROR;
}

void initPingPongStruct(pingpong_t *ptrPingPong, uint32_t maxSamples, uint32_t fifoLen, uint8_t sampleSize) {
    ptrPingPong->maxSamples = maxSamples;
    ptrPingPong->sampleSize = sampleSize;
    ptrPingPong->rtcErrorFlag = 0;
    ptrPingPong->Index = PING;
    ptrPingPong->fifoLen = fifoLen;
    ptrPingPong->Buffer[0] = malloc(maxSamples*sampleSize);
    ptrPingPong->Buffer[1] = malloc(maxSamples*sampleSize);
    ptrPingPong->Available[0] = 0;
    ptrPingPong->Available[1] = 0;
}

uint8_t *getCurPtr(pingpong_t *ptrPingPong) {
    uint32_t numAvailableSamples = ptrPingPong->Available[ptrPingPong->Index];
    uint8_t *curPtr = NULL;
    if((ptrPingPong->maxSamples - numAvailableSamples) > ptrPingPong->fifoLen) {
        //stay on current buffer
        curPtr = ptrPingPong->Buffer[ptrPingPong->Index] + numAvailableSamples*ptrPingPong->sampleSize;
    }
    else {
        //switch buffer
        ptrPingPong->rtcErrorFlag++;
        // printf("Current buffer %d has %d samples\r\n", pingpongDataIndex, pingpongDataBufferAvailable[pingpongDataIndex]);
        ptrPingPong->Index ^= 1;
        // printf("Switching to %d buffer\r\n", pingpongDataIndex);

        //Switch Buffer,reset write Pointer and available sample count
        curPtr = ptrPingPong->Buffer[ptrPingPong->Index];
        ptrPingPong->Available[ptrPingPong->Index] = 0;
    }
    return curPtr;
}

void updatePingPong(pingpong_t *ptrPingPong, uint32_t numSamples) {
    uint8_t index = ptrPingPong->Index;
    ptrPingPong->Available[index] += numSamples;
#if 0
    if(ptrPingPong->sampleSize == 4) {
        uint32_t available = ptrPingPong->Available[index];
        float *curPtr = (float *)ptrPingPong->Buffer[index];
        printf("Temp PingPong Avail %d, Index %d\r\n", available, index);
        printf("Temp ");
        for(int i = 0; i < available; i++) {
            printf("%3.1f,", *curPtr++);
        }
        printf("\r\n");
    }
#endif
}
        
void outputPingPong(pingpong_t *ptrPingPong, FILE *fp, uint8_t flush) {
    if((ptrPingPong->rtcErrorFlag > 0) ||  ((ptrPingPong->rtcErrorFlag == 0) && (flush == 1))) {
        if((ptrPingPong->rtcErrorFlag > 1) && (flush == 0)) {
            printf("RTC Error %d\n", ptrPingPong->rtcErrorFlag);
            reasonCode = RTC_ERROR;
            terminate(RTC_ERROR);
        }

        uint8_t index = PING;
        if(flush) {
            index = ptrPingPong->Index;
        }
        else {
            index = ptrPingPong->Index ^ 1;
        }

        if(flush) {
            uint32_t available = ptrPingPong->Available[index];
            printf("FLushing Temp PingPong Avail %d, Index %d\r\n", available, index);
        }

        uint8_t *sampleBuffer = ptrPingPong->Buffer[index];
        uint32_t numSamples = ptrPingPong->Available[index];
        fwrite(sampleBuffer, ptrPingPong->sampleSize, numSamples, fp);
        ptrPingPong->rtcErrorFlag--;
    }
}

void clearPingPong(pingpong_t *ptrPingPong) {
    free(ptrPingPong->Buffer[0]);
    free(ptrPingPong->Buffer[1]);
}
