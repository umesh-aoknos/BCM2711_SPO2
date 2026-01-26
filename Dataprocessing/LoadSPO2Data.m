function [irLED, redLED, Temp, setup, adcInfo, dacInfo, ppgConfig] = LoadSPO2Data(FileName, sensorPlacement, saveTextFlag)
    if(nargin == 1)
        sensorPlacement = "L FF";
        saveTextFlag = 0;
    elseif(nargin == 2)
        saveTextFlag = 0;
    end

    [dirName, baseName, extn] = fileparts(FileName);
    if(length(dirName) == 0)
        dirName = '.';
    end

    if(strfind(baseName, 'PPG'))
        TempFileName = strrep(baseName, 'PPG', 'Temp');
        TempFileName = strcat(dirName,'/',TempFileName, extn);

        DescFileName = strrep(baseName, 'PPG', 'desc');
        DescFileName = strcat(dirName,'/',DescFileName, '.json');
    else
        TempFileName = strrep(baseName, 'Data', 'Temp');
        TempFileName = strcat(dirName,'/',TempFileName, extn);

        DescFileName = '';
    end

    GlobalDefinitions

    if(length(DescFileName) == 0)
        [setup, adcInfo, dacInfo, ppgConfig] = LoadSessionInfoFromData(FileName, sensorPlacement);
    else
        [setup, adcInfo, dacInfo, ppgConfig] = LoadSessionInfoFromJSON(DescFileName);
    end

    fid = fopen(FileName, "r");
    skipBytes = 0;

    switch(setup.version)
        case 0
            skipBytes = 13;
        case 1
            skipBytes = 24;
        case 2
            skipBytes = 18;
    end
    if(skipBytes > 0)
        dummy = fread(fid, skipBytes, 'uint8');%%Read dummy to align with Data Start
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

    %%Read Temp
    fidTemp = fopen(TempFileName, "r");
    if(fidTemp > 0)
        if(setup.version >= 2)
            rawTempA = fread(fidTemp, Inf, 'uint8');
            N=floor(length(rawTempA)/2);
            rawTempB = reshape(rawTempA(1:N*2),2,[]);
            Temp = (rawTempB(1,:) + (rawTempB(2,:)*.0625/16));
        else
            Temp = fread(fidTemp, Inf, 'float');
        end

        fclose(fidTemp);
    else
        Temp = [];
    end

    if(saveTextFlag == 1)
        saveDataTextFile(FileName, irLED, redLED, config);
    end
end
