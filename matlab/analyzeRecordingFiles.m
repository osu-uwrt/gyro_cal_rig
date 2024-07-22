%
% Gyro calibration data analysis function.
% Up to 16GB of ram or swap space is required for this script to run,
% depending on the amount of data being processed
%
function stepsPerRotation = analyzeRecordingFiles(files)
    format compact
    
    rawdata = []; % order: caltimes, calgyrorates, calrigrates, caltemps
    numRotations = input("Enter total number of rotations: ");
    
    for i = 1 : length(files)
        filename = files{i};
        fprintf("Loading table %d/%d (%s)\n", i, length(files), filename);
        table = readtable(filename, "NumHeaderLines", 1);
    
        %header: sec,nanosec,rig_rate
    
        tabletimes = table{:, 1} + (table{:, 2} / 1000000000) - table{1, 1};
        tablerigrates = table{:, 3};
    
        rawdata = [rawdata; tabletimes, tablerigrates];
    end
    
    fprintf("Processing data\n");
    stepsTravelled = trapz(rawdata(:, 1), rawdata(:, 2));
    stepsPerRotation = stepsTravelled / numRotations;
    
    fprintf("Creating plots\n");
    
    figure;
    tiledlayout(1, 2);
    
    nexttile;
    plot(rawdata(:, 1));
    title("Time");
    ylabel("Time (s)");

    nexttile;
    plot(rawdata(:, 1), rawdata(:, 2));
    title("Rig rate vs Time");
    xlabel("Time (s)");
    ylabel("Rig rate (steps / s)");
end
