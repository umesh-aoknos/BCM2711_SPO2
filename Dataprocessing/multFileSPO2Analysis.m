function multFileSPO2Analysis(irLED, redLED, Temp, version, infoPPG, skipSamplesDur, NumFiles, FileIdx)
    GlobalDefinitions
    if(nargin == 5)
        skipSamplesDur = 0;%%sec
        NumFiles = 1;
        FileIdx = 1;
    elseif(nargin == 6)
        NumFiles = 1;
        FileIdx = 1;
    elseif(nargin == 7)
        FileIdx = 1;
    end

    infoPPG.sensorPlacement = PositionsArray{FileIdx};
    NumCols = 3;
    reflected = 1;
    fs = infoPPG.ppgSampleRate;
    ts = 1/fs;
    N = length(irLED);
    t = (0:N-1)*ts;
    skipIndex = max(ceil(skipSamplesDur*fs), 1);
    skipIndexRange = (skipIndex:N);

    ledScale = infoPPG.adcRange/(2^infoPPG.pulseWidth - 1);
   
    tmod = t(skipIndexRange);
    mean_redLED = mean(redLED(skipIndexRange));
    mean_irLED = mean(irLED(skipIndexRange));

    if(reflected)
        irLEDmod = mean_irLED -irLED(skipIndexRange);
        redLEDmod = mean_redLED - redLED(skipIndexRange);
    else
        irLEDmod = irLED(skipIndexRange) - mean_irLED;
        redLEDmod = redLED(skipIndexRange) - mean_redLED;
    end
    subplot(NumFiles, NumCols, (FileIdx-1)*NumCols + 1)
    plot(tmod, ledScale*irLEDmod);
    ylabel('IR Detector nA');
    xlabel('Time (seconds)');
    grid

    subplot(NumFiles, NumCols, (FileIdx-1)*NumCols + 2)
    plot(tmod, ledScale*redLEDmod, 'r');
    if(version == 1)
        titleStr = sprintf("%s. f_s=%dHz,i_{ir}=%2.1fma,i_{red}=%2.1fma, adc_{res}=%d, adc_{range}=%dnA, Die Temp=%2.1f(C)",infoPPG.sensorPlacement, fs,infoPPG.redLEDCurrent,infoPPG.redLEDCurrent, infoPPG.pulseWidth, infoPPG.adcRange, infoPPG.dieTemp);
    else
        titleStr = sprintf("%s. f_s=%dHz,i_{ir}=%2.1fma,i_{red}=%2.1fma, adc_{res}=%d, adc_{range}=%dnA.",infoPPG.sensorPlacement, fs,infoPPG.redLEDCurrent,infoPPG.redLEDCurrent,infoPPG.pulseWidth, infoPPG.adcRange);
    end
    title(titleStr)
    ylabel('Red Detector nA');
    xlabel('Time (seconds)');
    grid

    subplot(NumFiles, NumCols, (FileIdx-1)*NumCols + 3)
    plot(Temp)
    ylabel('Temp C\deg');
    grid
end
