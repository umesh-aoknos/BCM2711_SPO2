function [irLED, redLED, Temp, config] = processData(FileName, sensorPlacement, saveTextFlag)
    dataDir = '.';
    switch(nargin)
        case 0
            %% FileName = "saveData/SPO2Data_30s10mA.dat";
            %% FileName = "SPO2Data1768021089.dat";
            %% FileName = "SPO2Data1768281491.dat";
            searchStr = strcat(dataDir, '/*Data*.dat');
            list = dir(searchStr);
	    no_desc_idx = cellfun(@isempty, strfind({list.name}, '_desc'));
	    finalList = list(no_desc_idx); 
            FileName = finalList(end).name
            sensorPlacement = "Long L FF";
            saveTextFlag = 0;
        case 1
            sensorPlacement = "Long L FF";
            saveTextFlag = 0;
        case 2
            saveTextFlag = 0;
    end
    [irLED, redLED, Temp, config] = LoadSPO2Data(FileName, sensorPlacement, saveTextFlag);
    skipSamplesDur = 0.25;
    analyzeSPO2Data(irLED, redLED, Temp, config, skipSamplesDur);
end
