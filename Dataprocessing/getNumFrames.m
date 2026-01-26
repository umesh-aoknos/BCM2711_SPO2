function numFrames = getNumFrames(duration, adcInfo)
    numFrames = floor(duration*adcInfo.fsamp/adcInfo.Window);
end
