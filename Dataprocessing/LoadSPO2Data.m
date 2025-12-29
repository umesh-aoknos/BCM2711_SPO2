GlobalDefines
fid = fopen("SPO2Data.dat","r");
%%Read Config
config = struct(); % Initialize an empty structure
% Read the config fields
dataStruct.sample_avg = MAX30102GetConfig(fread(fid, 1, 'uint8'), SAMPLE_AVG);
dataStruct.fifo_rollover_en = MAX30102GetConfig(fread(fid, 1, 'uint8'), ROLLOVER_EN);
dataStruct.fifo_full_trigger = MAX30102GetConfig(fread(fid, 1, 'uint8'), FIFO_FULL_TRIGGER);
dataStruct.sample_rate = MAX30102GetConfig(fread(fid, 1, 'uint8'), SAMPLE_RATE);
dataStruct.pulse_width = MAX30102GetConfig(fread(fid, 1, 'uint8'), ADC_RES);
dataStruct.adc_range = MAX30102GetConfig(fread(fid, 1, 'uint8'),ADC_RANGE);
dataStruct.slot1 = MAX30102GetConfig(fread(fid, 1, 'uint8'), SLOT1_ASSIGN);
dataStruct.slot2 = MAX30102GetConfig(fread(fid, 1, 'uint8'), SLOT2_ASSIGN);
dataStruct.slot3 = MAX30102GetConfig(fread(fid, 1, 'uint8'), SLOT3_ASSIGN);
dataStruct.slot4 = MAX30102GetConfig(fread(fid, 1, 'uint8'), SLOT4_ASSIGN);
dataStruct.redled_current = MAX30102GetConfig(fread(fid, 1, 'uint8'), REDLED_CUR);
dataStruct.irled_current = MAX30102GetConfig(fread(fid, 1, 'uint8'), IRLED_CUR);
dataStruct.mode = MAX30102GetConfig(fread(fid, 1, 'uint8'), MODE);

sensorPlacement = "L FF";

dataStruct

%%Read Red/IR Sample Data
A = fread(fid, Inf, 'uint8');
N=floor(length(A)/6)
size(A)
B = reshape(A(1:N*6),6,[]);
S = size(B)
redLED = (B(1,:)&0x03)*2^16 + B(2,:)*2^8 + B(3,:);
irLED = (B(4,:)&0x03)*2^16 + B(5,:)*2^8 + B(6,:);
%% redLED = B(3,:)*2^16 + B(2,:)*2^8 + B(1,:);
%% irLED = B(6,:)*2^16 + B(5,:)*2^8 + B(4,:);
fclose(fid)
fs = dataStruct.sample_rate;
ts = 1/fs;
N = S(2)
t = (0:N-1)*ts;
settlingTime = 1;%%sec
sampleIdx = (settlingTime*fs:N);
mean_redLED = mean(redLED(sampleIdx));
mean_irLED = mean(irLED(sampleIdx));
ax = plotyy(t(sampleIdx), irLED(sampleIdx)-mean_irLED, t(sampleIdx), redLED(sampleIdx)-mean_redLED);
xlabel('Time');
ylabel(ax(1), 'IR LED');
ylabel(ax(2), 'Red LED');
legend('IR LED', 'RED LED')
titleStr = sprintf("Place %s. Config: f_s=%dHz,i_{ir}=%2.1f(ma),i_{red}=%2.1f(ma), adc_{res}=%d",sensorPlacement, fs,dataStruct.redled_current,dataStruct.redled_current,dataStruct.pulse_width);
title(titleStr)
