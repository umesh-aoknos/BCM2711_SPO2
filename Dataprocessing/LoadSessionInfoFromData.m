function [setup, adcInfo, dacInfo, ppgConfig] = LoadSessionInfoFromData(configFileName, sensorPlacement)
    GlobalDefinitions
    fid = fopen(configFileName, "r");
    %%Read Config
    ppgConfig = struct(); % Initialize an empty structure
    % Read the config fields
    setup.version = MAX30102GetConfig(fread(fid, 1, 'uint16'), VERSION);
    if((setup.version  < 32769)) % If version is negative, it's old format
        fseek(fid, 0, 'bof'); 
        setup.version = 0;
    else
        setup.version = setup.version - 32768;
        ppgConfig.i2c_freq = MAX30102GetConfig(fread(fid, 1, 'uint16'), I2C_FREQ);
    end
    ppgConfig.i2cFreq = 1500;
    ppgConfig.sampleAvg = MAX30102GetConfig(fread(fid, 1, 'uint8'), SAMPLE_AVG);
    ppgConfig.fifoRolloverEn = MAX30102GetConfig(fread(fid, 1, 'uint8'), ROLLOVER_EN);
    ppgConfig.fifoFullTrigger = MAX30102GetConfig(fread(fid, 1, 'uint8'), FIFO_FULL_TRIGGER);
    ppgConfig.ppgSampleRate = MAX30102GetConfig(fread(fid, 1, 'uint8'), SAMPLE_RATE);
    ppgConfig.pulseWidth = MAX30102GetConfig(fread(fid, 1, 'uint8'), ADC_RES);
    ppgConfig.adcRange = MAX30102GetConfig(fread(fid, 1, 'uint8'),ADC_RANGE);
    ppgConfig.slot1 = MAX30102GetConfig(fread(fid, 1, 'uint8'), SLOT1_ASSIGN);
    ppgConfig.slot2 = MAX30102GetConfig(fread(fid, 1, 'uint8'), SLOT2_ASSIGN);
    ppgConfig.slot3 = MAX30102GetConfig(fread(fid, 1, 'uint8'), SLOT3_ASSIGN);
    ppgConfig.slot4 = MAX30102GetConfig(fread(fid, 1, 'uint8'), SLOT4_ASSIGN);
    ppgConfig.redLEDCurrent = MAX30102GetConfig(fread(fid, 1, 'uint8'), REDLED_CUR);
    ppgConfig.irLEDCurrent = MAX30102GetConfig(fread(fid, 1, 'uint8'), IRLED_CUR);
    ppgConfig.mode = MAX30102GetConfig(fread(fid, 1, 'uint8'), MODE);
    ppgConfig.sensorPlacement = sensorPlacement;
    ppgConfig.tempSampleRate = 5;
    switch(setup.version)
        case 1
            % If version is 1. Read dieTemp and then skip 3 bytes for align (24bytes vs 21 bytes of info)
            dummy = fread(fid, 3, 'uint8');%%Read dummy to align with dieTemp
            ppgConfig.dieTemp = fread(fid, 1, 'float');
        case 2
            % If version is 1. No dieTemp. Skip 3 bytes for align (18bytes vs 17 bytes of info)
            dummy = fread(fid, 1, 'uint8');%%Read dummy to align PPG Data
    end
    fclose(fid);
    adcInfo = {};
    dacInfo = {};
end

