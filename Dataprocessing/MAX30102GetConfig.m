function parsedValue = MAX30102GetConfig(val, field)
    GlobalDefinitions
    switch (field)
        case VERSION
            if(val == 0xFFFF)
                parsedValue = -1; % Indicate old format
            else
                parsedValue = val;
            end
        case I2C_FREQ
            parsedValue = 150e6 /(val*1e3);%%150MHz/CDIV
        case SAMPLE_AVG
            parsedValue = 2^val;
        case SAMPLE_RATE
            parsedValue = MAX30102_FS_LUT(val + 1);
        case ADC_RANGE
            parsedValue = MAX30102_ADCRGE_LUT(val + 1);
        case ADC_RES
            parsedValue = MAX30102_ADCRES_LUT(val + 1);
        case {SLOT1_ASSIGN,SLOT2_ASSIGN,SLOT3_ASSIGN,SLOT4_ASSIGN}
            parsedValue = MAX30102_SLOT_LUT{val + 1};
        case MODE
            parsedValue = MAX30102_MODE_LUT(val + 1);
        case {IRLED_CUR, REDLED_CUR}
            parsedValue = val*0.2;
        otherwise
            parsedValue = val;
    end
end
