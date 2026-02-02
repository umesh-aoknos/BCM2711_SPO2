function [irLED, redLED, Temp, setup, adcInfo, dacInfo, ppgConfig] = processData(FileName, sensorPlacement, saveTextFlag)
    dataDir = '.';
    switch(nargin)
        case 0
            %% FileName = "saveData/SPO2Data_30s10mA.dat";
            %% FileName = "SPO2Data1768021089.dat";
            %% FileName = "SPO2Data1768281491.dat";
            searchStr = strcat(dataDir, '/*PPG*.dat');
            list = dir(searchStr);
            if(length(list) > 0)
                no_desc_idx = cellfun(@isempty, strfind({list.name}, '_desc'));
                finalList = list(no_desc_idx); 
            else
                searchStr = strcat(dataDir, '/*Data*.dat');
                finalList = dir(searchStr);
            end
            FileName = finalList(end).name;
            sensorPlacement = "Long L FF";
            saveTextFlag = 0;
        case 1
            sensorPlacement = "Long L FF";
            saveTextFlag = 0;
        case 2
            saveTextFlag = 0;
    end
    [irLED, redLED, Temp, setup, adcInfo, dacInfo, ppgConfig] = LoadSPO2Data(FileName, sensorPlacement, saveTextFlag);
    skipSamplesDur = 0.5;
    analyzeSPO2Data(irLED, redLED, Temp, setup.version, ppgConfig, skipSamplesDur);
end
