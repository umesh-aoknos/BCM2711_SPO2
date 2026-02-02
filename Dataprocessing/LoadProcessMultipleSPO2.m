dataDir = '.';
%% dataDir = '/Users/umesh/Projects/AoknosGitHub/SPO2SavedData/ADCRangeAnalysis/20260129';
searchStr = strcat(dataDir, '/*Data*.dat')
fileList = dir(searchStr);

sensorPlacement = 'L FF';
saveTextFlag = 0;
skipSamplesDur = 0.5;
NumFiles = length(fileList);
for fileIdx = 1:NumFiles
    FileName = fileList(fileIdx).name;
    FileName = strcat(dataDir, '/', FileName)
    [irLED, redLED, Temp, setup, adcInfo, dacInfo, ppgConfig] = LoadSPO2Data(FileName, sensorPlacement, saveTextFlag);
    multFileSPO2Analysis(irLED, redLED, Temp, setup.version, ppgConfig, skipSamplesDur, NumFiles, fileIdx);
end
