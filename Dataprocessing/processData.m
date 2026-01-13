%% FileName = "saveData/SPO2Data_30s10mA.dat";
%% FileName = "SPO2Data1768021089.dat";
FileName = "SPO2Data1768020927.dat";
sensorPlacement = "Long L FF";
saveTextFlag = 0;
[irLED, redLED, config] = LoadSPO2Data(FileName, sensorPlacement, saveTextFlag);
skipSamplesDur = 0.25;
analyzeSPO2Data(irLED, redLED, config, skipSamplesDur);
