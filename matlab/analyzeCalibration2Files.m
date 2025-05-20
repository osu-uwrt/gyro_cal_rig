%
% Gyro calibration v2 data analysis function.
% Up to 16GB of ram or swap space is required for this script to run,
% depending on the amount of data being processed
%
function [fit, gof, filtereddata] = analyzeCalibration2Files(files)
    format compact
    
    STEPS_PER_ROTATION = 577803.965671;
    RADS_PER_STEP = (1 / STEPS_PER_ROTATION) * 2 * pi;
    SEGMENT_MARGIN = 50; % this number of points is clipped from both sides of each segment
    
    fprintf("Populating arrays\n");
    
    rawcaldata = []; % order: caltimes, calgyrorates, calrigrates, caltemps
    
    for i = 1 : length(files)
        filename = files{i};
        fprintf("Loading table %d/%d (%s)\n", i, length(files), filename);
        table = readtable(filename, "NumHeaderLines", 1);
    
        %header: sec,nanosec,gyro_raw,gyro_temp,rig_rate,rig_heating,rig_enabled,rig_stalled
    
        tabletimes = table{:, 1} + (table{:, 2} / 1000000000) - table{1, 1};
        tablegyrorates = table{:, 5};
        tabletemps = table{:, 3};
        tablerigrates = table{:, 4} * RADS_PER_STEP;
    
        rawcaldata = [rawcaldata; tabletimes, tablegyrorates, tablerigrates, tabletemps];
    end
    
    %process data
    fprintf("Processing data\n");
    
    splitlocs = abs(diff(rawcaldata(:, 3))) > 0;
    numsegments = sum(splitlocs) + 1;
    filtereddata = zeros(numsegments, 3);

    rawbuffer = rawcaldata;

    for i = 1 : numsegments
        fprintf("Processing segment %d / %d\n", i, numsegments)
        segmentdata = rawbuffer(1 : find(splitlocs), :);
        
        %cut margin
        segmentdata = segmentdata(SEGMENT_MARGIN : length(segmentdata) - SEGMENT_MARGIN, :);

        segmentrigratemean = mean(segmentdata(:, 3));
        segmentgyroratemean = mean(segmentdata(:, 2));
        segmenttempmean = mean(segmentdata(:, 4));
        filtereddata (i, :) = [segmentrigratemean, segmentgyroratemean, segmenttempmean];

        rawbuffer = rawbuffer(find(splitlocs) + 1 : length(rawbuffer), :);
        splitlocs = splitlocs(find(splitlocs) + 1 : length(splitlocs));
    end
       
    fprintf("Removing NaNs\n");
    filtereddata(isnan(filtereddata(:, 1)), :) = [];

    fprintf("Normalizing gyro rate data\n");
    gyroratemean = mean(filtereddata(:, 2));
    gyroratestd = std(filtereddata(:, 2));
    filtereddata(:, 2) = (filtereddata(:, 2) - gyroratemean) / gyroratestd;

    fprintf("Normalizing gyro temperature data\n");
    gyrotempmean = mean(filtereddata(:, 3));
    gyrotempstd = std(filtereddata(:, 3));
    filtereddata(:, 3) = (filtereddata(:, 3) - gyrotempmean) / gyrotempstd;

    %plot data
    fprintf("Creating plots\n");

    figure;
    tiledlayout(3, 2);
    nexttile;
    plot(rawcaldata(:, 1), '.');
    title("Time");
    ylabel("Time (s)");

    nexttile;
    plot(rawcaldata(:, 1), rawcaldata(:, 3), '.');
    title("Rig rate vs time");
    xlabel("Time (s)");
    ylabel("Rig rate (rad/s)");

    nexttile;
    plot(rawcaldata(:, 1), rawcaldata(:, 2), '.');
    title("Gyro rate vs time");
    xlabel("Time (s)");
    ylabel("Gyro rate");

    nexttile;
    plot(rawcaldata(:, 1), rawcaldata(:, 4), '.');
    title("Gyro temperature vs time");
    xlabel("Time (s)");
    ylabel("Gyro temperature (c)");

    nexttile;
    plot(filtereddata(:, 1), filtereddata(:, 2), '.');
    title("Filtered gyro rate vs Filtered rig rate");
    xlabel("Filtered Rig rate (rads/sec)");
    ylabel("Filtered Gyro rate");
    
    fprintf("Creating fit\n");
    [fit, gof] = fitcalibration2data(filtereddata(:, 2), filtereddata(:, 3), filtereddata(:, 1));

    %display temp limits
    fprintf("Rate normalized by mean %d and std %d\n", gyroratemean, gyroratestd);
    fprintf("Temp normalized by mean %d and std %d\n", gyrotempmean, gyrotempstd);
    fprintf("Temperature min: %f\n", min(rawcaldata(:, 4)));
    fprintf("Temperature max: %f\n", max(rawcaldata(:, 4)));
end
