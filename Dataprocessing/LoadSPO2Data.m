function [irLED, redLED, config] = LoadSPO2Data(FileName, sensorPlacement, saveTextFlag)
    if(nargin == 1)
        sensorPlacement = "L FF";
        saveTextFlag = 0;
    elseif(nargin == 2)
        saveTextFlag = 0;
    end

    GlobalDefines
    fid = fopen(FileName, "r");
    %%Read Config
    config = struct(); % Initialize an empty structure
    % Read the config fields
    config.version = MAX30102GetConfig(fread(fid, 1, 'uint16'), VERSION)
    if((config.version  < 32769)) % If version is negative, it's old format
        fseek(fid, 0, 'bof'); 
        config.version = 0;
    else
        config.version = config.version - 32768;
        config.i2c_freq = MAX30102GetConfig(fread(fid, 1, 'uint16'), I2C_FREQ)
    end
    config.sample_avg = MAX30102GetConfig(fread(fid, 1, 'uint8'), SAMPLE_AVG)
    config.fifo_rollover_en = MAX30102GetConfig(fread(fid, 1, 'uint8'), ROLLOVER_EN);
    config.fifo_full_trigger = MAX30102GetConfig(fread(fid, 1, 'uint8'), FIFO_FULL_TRIGGER);
    config.sample_rate = MAX30102GetConfig(fread(fid, 1, 'uint8'), SAMPLE_RATE);
    config.pulse_width = MAX30102GetConfig(fread(fid, 1, 'uint8'), ADC_RES);
    config.adc_range = MAX30102GetConfig(fread(fid, 1, 'uint8'),ADC_RANGE);
    config.slot1 = MAX30102GetConfig(fread(fid, 1, 'uint8'), SLOT1_ASSIGN);
    config.slot2 = MAX30102GetConfig(fread(fid, 1, 'uint8'), SLOT2_ASSIGN);
    config.slot3 = MAX30102GetConfig(fread(fid, 1, 'uint8'), SLOT3_ASSIGN);
    config.slot4 = MAX30102GetConfig(fread(fid, 1, 'uint8'), SLOT4_ASSIGN);
    config.redled_current = MAX30102GetConfig(fread(fid, 1, 'uint8'), REDLED_CUR);
    config.irled_current = MAX30102GetConfig(fread(fid, 1, 'uint8'), IRLED_CUR);
    config.mode = MAX30102GetConfig(fread(fid, 1, 'uint8'), MODE);
    config.sensorPlacement = sensorPlacement;
    if(config.version  > 0) % If version is present then adjust for struct size align (18byts vs 17bytes of info)
        dummy = fread(fid, 3, 'uint8');%%Read dummy to align with dieTemp
        config.dieTemp = fread(fid, 1, 'float');
    end

    %%Read Red/IR Sample Data
    A = fread(fid, Inf, 'uint8');
    N=floor(length(A)/6);
    B = reshape(A(1:N*6),6,[]);
    redLED = (B(1,:)&0x03)*2^16 + B(2,:)*2^8 + B(3,:);
    irLED = (B(4,:)&0x03)*2^16 + B(5,:)*2^8 + B(6,:);
    %% redLED = B(3,:)*2^16 + B(2,:)*2^8 + B(1,:);
    %% irLED = B(6,:)*2^16 + B(5,:)*2^8 + B(4,:);
    fclose(fid);

    if(saveTextFlag == 1)
        saveDataTextFile(FileName, irLED, redLED, config);
    end
end
