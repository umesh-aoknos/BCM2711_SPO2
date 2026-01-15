function analyzeSPO2Data(irLED, redLED, Temp, config, skipSamplesDur)
    if(nargin == 3)
        skipSamplesDur = 0;%%sec
    end
    reflected = 1;
    fs = config.sample_rate;
    ts = 1/fs;
    N = length(irLED);
    t = (0:N-1)*ts;
    %fc = 0.2;
    %[B, A] = butter(4, 2*fc/fs);
    %% irLED = filter(B, A, irLED);
    %% redLED = filter(B, A, redLED);
    skipIndex = max(ceil(skipSamplesDur*fs), 1);
    skipIndexRange = (skipIndex:N);
   
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
    
    figure(1)
    axfig1 = plotyy(tmod, irLEDmod, tmod, redLEDmod);
    xlabel('Time');
    ylabel(axfig1(1), 'IR LED');
    ylabel(axfig1(2), 'Red LED');
    legend('IR LED', 'RED LED')
    if(config.version > 0)
        titleStr = sprintf("Place %s. Config: f_s=%dHz,i_{ir}=%2.1f(ma),i_{red}=%2.1f(ma), adc_{res}=%d, Die Temp=%2.1f(C)",config.sensorPlacement, fs,config.redled_current,config.redled_current,config.pulse_width, config.dieTemp);
    else
        titleStr = sprintf("Place %s. Config: f_s=%dHz,i_{ir}=%2.1f(ma),i_{red}=%2.1f(ma), adc_{res}=%d.",config.sensorPlacement, fs,config.redled_current,config.redled_current,config.pulse_width);
    end
    title(titleStr)
    grid
    
    figure(2)
    NFFT = 1024;
    F = (0:NFFT-1)*fs*60/NFFT;%%bpm
    irLEDFreqMag = abs(fft(irLEDmod, NFFT));
    redLEDFreqMag = abs(fft(redLEDmod, NFFT));
    axfig2 = plotyy(F(1:NFFT/2+1), 10*log10(irLEDFreqMag(1:NFFT/2 + 1)), F(1:NFFT/2+1), 10*log10(redLEDFreqMag(1:NFFT/2 + 1)));
    [sortVal, sortIdx] = sort(irLEDFreqMag(1:NFFT/2 + 1), 'descend');
    axis(axfig2(1), [0,2*60,30,56]);
    axis(axfig2(2), [0,2*60,30,56]);
    title('Freq Analysis')
    ylabel('LED Spectral Magn')
    xlabel('Freq (bmp)')
    legend('IR LED', 'RED LED')
    grid

    if(!isempty(Temp))
        figure(3)
        plot(Temp)
        ylabel('Temp C\deg');
        title('Temperature');
        grid
    end
end
