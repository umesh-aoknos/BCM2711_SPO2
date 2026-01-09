function saveDataTextFile(FileName, irLED, redLED, config)
    [dirName, baseName, ~] = fileparts(FileName)
    if(length(dirName) == 0)
        dirName = ".";
    end

    % Generate output filenames automatically
    fid = fopen(FileName, "r");
    irFile = sprintf('%s//%s_irLED.txt', dirName, baseName)
    redFile = sprintf('%s//%s_redLED.txt', dirName, baseName)
    bothFile = sprintf('%s//%s_both_signals.txt', dirName, baseName)
    
    % Remove Mean
    mean_redLED = mean(redLED);
    mean_irLED = mean(irLED);
    redLED = redLED - mean_redLED;
    irLED = irLED - mean_irLED;

    % Save IR LED signal
    dlmwrite(irFile, irLED, '\t');
    fprintf('IR LED data saved to: %s\n', irFile);
    
    % Save Red LED signal
    dlmwrite(redFile, redLED, '\t');
    fprintf('Red LED data saved to: %s\n', redFile);
    
    % Save both signals with metadata
    fid_out = fopen(bothFile, 'w');
    fprintf(fid_out, '%% Sampling rate: %d Hz\n', config.sample_rate);
    fprintf(fid_out, '%% Samples: %d\n', length(irLED));
    fprintf(fid_out, '%% IR_LED\tRed_LED\n');
    fprintf(fid_out, '%f\t%f\n', [irLED; redLED]);
    fclose(fid_out);
    fprintf('Both signals saved to: %s\n', bothFile);
end
