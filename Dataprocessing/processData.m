%% FileName = "saveData/SPO2Data_30s10mA.dat";
FileName = "SPO2Data1767885855.dat";
sensorPlacement = "L FF";
saveTextFlag = 1;
[irLED, redLED, config] = LoadSPO2Data(FileName, sensorPlacement, saveTextFlag);
skipSamplesDur = 0.25;
analyzeSPO2Data(irLED, redLED, config, skipSamplesDur)
