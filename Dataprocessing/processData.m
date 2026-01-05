[irLED, redLED, config] = LoadSPO2Data('SPO2Data1767585152.dat', "L FF");
fs = config.sample_rate;
ts = 1/fs;
N = length(irLED);
t = (0:N-1)*ts;
fc = 0.2;
[B, A] = butter(4, 2*fc/fs);
%% irLED = filter(B, A, irLED);
%% redLED = filter(B, A, redLED);
settlingTime = 1;%%sec
sampleIdx = (settlingTime*fs:N);
L = length(sampleIdx);
mean_redLED = mean(redLED(sampleIdx));
mean_irLED = mean(irLED(sampleIdx));
axfig1 = plotyy(t(sampleIdx), irLED(sampleIdx)-mean_irLED, t(sampleIdx), redLED(sampleIdx)-mean_redLED);
xlabel('Time');
ylabel(axfig1(1), 'IR LED');
ylabel(axfig1(2), 'Red LED');
legend('IR LED', 'RED LED')
titleStr = sprintf("Place %s. Config: f_s=%dHz,i_{ir}=%2.1f(ma),i_{red}=%2.1f(ma), adc_{res}=%d",config.sensorPlacement, fs,config.redled_current,config.redled_current,config.pulse_width);
title(titleStr)
grid

figure
NFFT = 1024;
F = (0:NFFT-1)*fs*60/NFFT;%%bpm
irLEDFreqMag = abs(fft(irLED(sampleIdx)-mean_irLED, NFFT));
redLEDFreqMag = abs(fft(redLED(sampleIdx)-mean_redLED, NFFT));
axfig2 = plotyy(F(1:NFFT/2+1), 10*log10(irLEDFreqMag(1:NFFT/2 + 1)), F(1:NFFT/2+1), 10*log10(redLEDFreqMag(1:NFFT/2 + 1)));
[sortVal, sortIdx] = sort(irLEDFreqMag(1:NFFT/2 + 1), 'descend');
axis(axfig2(1), [0,2*60,30,56]);
axis(axfig2(2), [0,2*60,30,56]);
title('Freq Analysis')
ylabel('LED Spectral Magn')
xlabel('Freq (bmp)')
legend('IR LED', 'RED LED')
grid
