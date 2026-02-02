function [setup, adcInfo, dacInfo, infoPPG] = LoadSessionInfoFromJSON(descFileJSON)
    GlobalDefinitions
    jsonText = fileread(descFileJSON);
    jsonData = jsondecode(jsonText);

    setup.version = double(jsonData.version);
    setup.numberOfStages = jsonData.numberOfStages;
    setup.captureDurationArray = jsonData.captureDurationArray;
    setup.delayBetweenStage = jsonData.delayBetweenStage;

    setup.fspi_divider = jsonData.fspi_divider;
    setup.fspi_freqMHz = 500/setup.fspi_divider;
    setup.captureTimeStamp = jsonData.captureTimeStamp;
    setup.infoStr = '';

    setup.hostName = jsonData.hostName;
    setup.hostPort = jsonData.hostPort;
    setup.metaStr = jsonData.metaStr;

    if(isfield(jsonData, 'acFreq'))
        setup.ac_freq = jsonData.acFreq;
    else
        setup.ac_freq = 55;
    end

    Str = sprintf("Session Info\n");
    setup.infoStr = strcat(setup.infoStr, Str);
    Str = sprintf("\tStart Capture %s", ctime(setup.captureTimeStamp));%%ctime has \n in its return
    setup.infoStr = strcat(setup.infoStr, Str);
    Str = sprintf("\tStage Setup: %d Stages of duration %d secs and a delay of %d secs between stages\n", setup.numberOfStages, setup.captureDurationArray(1), setup.delayBetweenStage);
    setup.infoStr = strcat(setup.infoStr, Str);
    Str = sprintf("\tSPI Freq: %5.2f (MHz)\n", setup.fspi_freqMHz);
    setup.infoStr = strcat(setup.infoStr, Str);
    Str = sprintf("\tUDP Host: %s:%d\n", setup.hostName, setup.hostPort);
    setup.infoStr = strcat(setup.infoStr, Str);

    Str = sprintf("\tMeta String: %s\n", setup.metaStr);
    setup.infoStr = strcat(setup.infoStr, Str);

    %%ADCInfo
    adcInfo.adcType = jsonData.infoADC.adcType;
    adcInfo.adcResolution = jsonData.infoADC.adcResolution;
    idx = find(ADCWordSizeArray - adcInfo.adcResolution > 0);
    adcInfo.ADCNumWordsPerSample = ADCWordSizeArray(idx)/numBitsPerByte;
    adcInfo.Window = UDP_PAYLOAD/adcInfo.ADCNumWordsPerSample;
    adcInfo.fsamp = jsonData.infoADC.fsamp;
    adcInfo.ADCVdd = jsonData.infoADC.VDDRef;
    setup.numFramesSetupStage = getNumFrames(setup.delayBetweenStage, adcInfo);
    for idx = 1:3
        lcf = length(setup.captureDurationArray);
        if(lcf > 1)
            setup.numFramesCaptureStage(idx) = getNumFrames(setup.captureDurationArray(idx), adcInfo);
        else
            setup.numFramesCaptureStage(idx) = getNumFrames(setup.captureDurationArray, adcInfo);
        end
    end
    Str = sprintf("\tADC Info: Device %s. Resolution %d, Frequency %d (Hz), Vdd %3.2f (Volts)\n", ADCDevices{adcInfo.adcType}, adcInfo.adcResolution, adcInfo.fsamp, adcInfo.ADCVdd);
    setup.infoStr = strcat(setup.infoStr, Str);
    %%DAC1 Info
    dac1Info.dacType = jsonData.infoDAC1.dacType;
    dac1Info.dacResolution = jsonData.infoDAC1.dacResolution;
    dac1Info.dacVdd = jsonData.infoDAC1.VDDRef;
    dac1Info.dacamplitude = jsonData.infoDAC1.amplitude;
    dac1Info.dacfrequency = jsonData.infoDAC1.frequency;
    dac1Info.dacWaveform = jsonData.infoDAC1.waveform;
    dac1Info.dacModfrequency = jsonData.infoDAC1.fmod;
    dac1Info.dacMaxFrequencyDev = jsonData.infoDAC1.maxFreqDev;

    Str = sprintf("\tDAC1 Info: Device %s. Resolution %d, Amplitude %3.2f, Frequency %5.2f (Hz), Waveform %s, Vdd %3.2f (Volts)\n", DACDevices{dac1Info.dacType}, dac1Info.dacResolution, dac1Info.dacamplitude, dac1Info.dacfrequency, DACWaveForms{dac1Info.dacWaveform+1}, dac1Info.dacVdd);
    setup.infoStr = strcat(setup.infoStr, Str);
    if(dac1Info.dacWaveform > 3)
        Str = sprintf("\t\tDAC1 Modulation Info: Mod Frequency %5.2f (Hz), Max Frequency Deviation %5.2f (Hz)\n", dac1Info.dacModfrequency, dac1Info.dacMaxFrequencyDev);
        setup.infoStr = strcat(setup.infoStr, Str);
    end
    %%DAC2 Info
    dac2Info.dacType = jsonData.infoDAC2.dacType;
    dac2Info.dacResolution = jsonData.infoDAC2.dacResolution;
    dac2Info.dacVdd = jsonData.infoDAC2.VDDRef;
    dac2Info.dacamplitude = jsonData.infoDAC2.amplitude;
    dac2Info.dacfrequency = jsonData.infoDAC2.frequency;
    dac2Info.dacWaveform = jsonData.infoDAC2.waveform;
    dac2Info.dacModfrequency = jsonData.infoDAC2.fmod;
    dac2Info.dacMaxFrequencyDev = jsonData.infoDAC2.maxFreqDev;

    Str = sprintf("\tDAC2 Info: Device %s. Resolution %d, Amplitude %3.2f, Frequency %5.2f (Hz), Waveform %s, Vdd %3.2f (Volts)\n", DACDevices{dac2Info.dacType}, dac2Info.dacResolution, dac2Info.dacamplitude, dac2Info.dacfrequency, DACWaveForms{dac2Info.dacWaveform+1}, dac2Info.dacVdd);
    setup.infoStr = strcat(setup.infoStr, Str);
    if(dac2Info.dacWaveform > 3)
        Str = sprintf("\t\tDAC2 Modulation Info: Mod Frequency %5.2f (Hz), Max Frequency Deviation %5.2f (Hz)\n", dac2Info.dacModfrequency, dac2Info.dacMaxFrequencyDev);
        setup.infoStr = strcat(setup.infoStr, Str);
    end
    dacInfo = [dac1Info; dac2Info];

    %%PPG Info
    infoPPG.i2cFreq = jsonData.infoPPG.i2cFreq;
    infoPPG.sampleAvg = MAX30102GetConfig(jsonData.infoPPG.sampleAvg, SAMPLE_AVG);
    infoPPG.fifoRolloverEn = MAX30102GetConfig(jsonData.infoPPG.fifoRolloverEn, ROLLOVER_EN);
    infoPPG.fifoFullTrigger = MAX30102GetConfig(jsonData.infoPPG.fifoFullTrigger, FIFO_FULL_TRIGGER);
    infoPPG.ppgSampleRate = jsonData.infoPPG.ppgSampleRate
    %% infoPPG.pulseWidth = MAX30102GetConfig(jsonData.infoPPG.pulseWidth, ADC_RES);
    infoPPG.pulseWidth = jsonData.infoPPG.pulseWidth;
    infoPPG.adcRange = MAX30102GetConfig(jsonData.infoPPG.adcRange, ADC_RANGE);
    infoPPG.slot1 = MAX30102GetConfig(jsonData.infoPPG.slot1, SLOT1_ASSIGN);
    infoPPG.slot2 = MAX30102GetConfig(jsonData.infoPPG.slot2, SLOT2_ASSIGN);
    infoPPG.slot3 = MAX30102GetConfig(jsonData.infoPPG.slot3, SLOT3_ASSIGN);
    infoPPG.slot4 = MAX30102GetConfig(jsonData.infoPPG.slot4, SLOT4_ASSIGN);
    infoPPG.redLEDCurrent = jsonData.infoPPG.redLEDCurrent;
    infoPPG.irLEDCurrent = jsonData.infoPPG.irLEDCurrent;
    infoPPG.mode = MAX30102GetConfig(jsonData.infoPPG.mode, MODE);
    infoPPG.tempSampleRate = jsonData.infoPPG.tempSampleRate;
    infoPPG.sensorPlacement = 'L FF';
    %% infoPPG.sensorPlacement = jsonData.infoPPG.sensorPlacement;
end

