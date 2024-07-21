%
% Gyro calibration data analysis function.
% Up to 16GB of ram or swap space is required for this script to run,
% depending on the amount of data being processed
%
function [fitresult, gof] = analyzeCalibration(files)
    format compact
    
    STEPS_PER_ROTATION = 7.557241247324732e+05;
    RADS_PER_STEP = (1 / STEPS_PER_ROTATION) * 2 * pi;
    
    
    fprintf("Populating arrays\n");
    
    rawcaldata = []; % order: caltimes, calgyrorates, calrigrates, caltemps
    
    for i = 1 : length(files)
        filename = files{i};
        fprintf("Loading table %d/%d (%s)\n", i, length(files), filename);
        table = readtable(filename, "NumHeaderLines", 1);
    
        %header: sec,nanosec,gyro_raw,gyro_temp,rig_rate,rig_heating,rig_enabled,rig_stalled
    
        tabletimes = table{:, 1} + (table{:, 2} / 1000000000) - table{1, 1};
        tablegyrorates = table{:, 3};
        tabletemps = table{:, 4};
        tablerigrates = table{:, 5} * RADS_PER_STEP;
    
        rawcaldata = [rawcaldata; tabletimes, tablegyrorates, tablerigrates, tabletemps];
    end
    
    fprintf("Processing data\n");
    filteredcaldata = []; %same order as raw data
    uniquerigates = unique(rawcaldata(:, 3));
    numuniquerates = length(uniquerigates);    
    for i = 1 : numuniquerates
        rigrate = uniquerigates(i);
        fprintf("Processing rate %d / %d\n", i, numuniquerates);
        
        ratelocs = rawcaldata(:, 3) == rigrate;
        dataatrate = rawcaldata(ratelocs, :);
        gyrorates = dataatrate(:, 2);
        
        %remove outliers
        ratemean = mean(gyrorates);
        ratestdev = std(gyrorates);
        ratestokeep = abs(gyrorates - ratemean) < 3 * ratestdev;
        filteredcaldata = [filteredcaldata; dataatrate(ratestokeep, :)];
    end

    % negate the rig rates because the gyro is oriented upside-down
    filteredcaldata(:, 3) = filteredcaldata(:, 3) * -1;

    
    fprintf("Creating plots\n");
    
    figure;
    tiledlayout(3, 2);
    nexttile;
    plot(rawcaldata(:, 1));
    title("Time");
    ylabel("Time (s)");
    
    nexttile;
    plot(rawcaldata(:, 1), rawcaldata(:, 2), ".");
    title("Raw counts vs Time");
    xlabel("Time (s)")
    ylabel("Gyro rate (counts)");
    
    nexttile;
    plot(rawcaldata(:, 1), rawcaldata(:, 4), ".");
    title("Gyro temperature vs time");
    xlabel("Time (s)");
    ylabel("Gyro temperature (counts)");
    
    nexttile;
    plot(rawcaldata(:, 1), rawcaldata(:, 3), ".");
    title("Rig rate vs time");
    xlabel("Time (s)");
    ylabel("Rig rate (rads / sec)");
    
    nexttile;
    plot(filteredcaldata(:, 3), filteredcaldata(:, 2), ".");
    title("Gyro rate vs rig rate");
    xlabel("Rig rate (rads / sec)");
    ylabel("Gyro rate (counts)");
    
    nexttile
    plot(filteredcaldata(:, 2), filteredcaldata(:, 4), ".");
    title("Gyro temperature vs gyro raw rate");
    xlabel("Gyro temperature (counts)");
    ylabel("Gyro temperature (counts)");
    
    fprintf("Creating fit\n");
    [fitresult, gof] = fitcalibrationdata(filteredcaldata(:, 2), filteredcaldata(:, 4), filteredcaldata(:, 3));

    %display temp limits
    fprintf("Temperature min: %f\n", min(filteredcaldata(:, 4)));
    fprintf("Temperature max: %f\n", max(filteredcaldata(:, 4)));
end
