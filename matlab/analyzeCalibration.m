%
% Gyro calibration data analysis function
%

STEPS_PER_ROTATION = 7.557241247324732e+05;
RADS_PER_STEP = (1 / STEPS_PER_ROTATION) * 2 * pi;

fprintf("Please select input data\n");
[files, bases] = uigetfile("*.csv", "Select data files", "MultiSelect", "on");
fullfiles = fullfile(bases, files);

fprintf("Populating arrays\n");

caltimes = [];
calgyrorates = [];
calrigrates = [];
caltemps = [];

for i = 1 : length(fullfiles)
    filename = fullfiles{i};
    fprintf("Loading table %d/%d (%s)\n", i, length(fullfiles), filename);
    table = readtable(filename, "NumHeaderLines", 1);

    %header: sec,nanosec,gyro_raw,gyro_temp,rig_rate,rig_heating,rig_enabled,rig_stalled

    tabletimes = table{:, 1} + (table{:, 2} / 1000000000) - table{1, 1};
    tablegyrorates = table{:, 3};
    tabletemps = table{:, 4};
    tablerigrates = table{:, 5} * RADS_PER_STEP;

    caltimes = [caltimes; tabletimes];
    calgyrorates = [calgyrorates; tablegyrorates];
    calrigrates = [calrigrates; tablerigrates];
    caltemps = [caltemps; tabletemps];
end

fprintf("Processing data\n");
rawcaldata = [caltimes, calgyrorates, calrigrates, caltemps];
filteredcaldata = [];
uniquerigates = unique(calrigrates);
numuniquerates = size(uniquerigates);
for i = 1 : numuniquerates
    rigrate = uniquerigates(i);
    fprintf("Processing rate %d / %d\n", i, numuniquerates);
    
    ratelocs = calrigrates == rigrate;
    dataatrate = rawcaldata(ratelocs, :);
    gyrorates = dataatrate(:, 2);
    
    %remove outliers
    ratemean = mean(gyrorates);
    ratestdev = std(gyrorates);
    ratestokeep = abs(gyrorates - ratemean) < 3 * ratestdev;
    filteredcaldata = [filteredcaldata; dataatrate(ratestokeep, :)];
end

filteredtimes = filteredcaldata(:, 1);
filteredgyrorates = filteredcaldata(:, 2);
filteredrigrates = filteredcaldata(:, 3);
filteredtemps = filteredcaldata(:, 4);

fprintf("Creating plots\n");

figure;
tiledlayout(3, 2);
nexttile;
plot(caltimes);
title("Time");
ylabel("Time (s)");

nexttile;
plot(caltimes, calgyrorates);
title("Raw counts vs Time");
xlabel("Time (s)")
ylabel("Gyro rate (counts)");

nexttile;
plot(caltimes, caltemps, ".");
title("Gyro temperature vs time");
xlabel("Time (s)");
ylabel("Gyro temperature (counts)");

nexttile;
plot(caltimes, calrigrates, ".");
title("Rig rate vs time");
xlabel("Time (s)");
ylabel("Rig rate (rads / sec)");

nexttile;
plot(filteredrigrates, filteredgyrorates, ".");
title("Gyro rate vs rig rate");
xlabel("Rig rate (rads / sec)");
ylabel("Gyro rate (counts)");

nexttile
plot3(filteredgyrorates, filteredtemps, filteredrigrates, ".");
title("Raw calibration surface");
xlabel("Gyro rate (counts)");
ylabel("Gyro temperature (counts)");
zlabel("Actual rate (rads / sec)");

fprintf("Creating fit\n");
[fitresult, gof] = fitcalibrationdata(filteredgyrorates, filteredtemps, filteredrigrates);

%display fit as final act
fprintf("Final result: \n");
fitresult
