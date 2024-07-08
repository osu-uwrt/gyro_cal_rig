%
% Gyro calibration data analysis function
%

function calPoints = analyzeCalibration(logDir)
    format compact

    STEPPER_COUNTS_PER_REV = 240 * 256 * 10.8;

    gyroRawFile = fullfile(logDir, "gyro_raw.csv");
    gyroStatusFile = fullfile(logDir, "gyro_status.csv");
    rigStatusFile = fullfile(logDir, "rig.csv");
    
    gyroRawTable = readtable(gyroRawFile, "NumHeaderLines", 1);
    gyroStatusTable = readtable(gyroStatusFile, "NumHeaderLines", 1);
    rigStatusTable = readtable(rigStatusFile, "NumHeaderLines", 1);
    
    numPoints = height(gyroRawTable);
    calPoints = zeros(numPoints, 4);
    for readingIdx = 1 : numPoints
        gyroReading = gyroRawTable{readingIdx, :};
        sec = gyroReading(1);
        nanosec = gyroReading(2);
        gyroStatus = gyroStatusTable{findPointClosestToTime(gyroStatusTable, sec, nanosec), :};
        rigStatus = rigStatusTable{findPointClosestToTime(rigStatusTable, sec, nanosec), :};
        
        time = sec + nanosec/ 1e9;
        raw = gyroReading(3);
        rate = rigStatus(3) / STEPPER_COUNTS_PER_REV * 2 * pi;
        temp = gyroStatus(3);
        calPoints(readingIdx, :) = [time, rate, temp, raw];

        if mod(readingIdx, 1000) == 0
            fprintf("Processed %d / %d points (%f%%)\n", readingIdx, numPoints, readingIdx / numPoints * 100);
        end
    end
    
    figure;
    plot(calPoints(:, 1), calPoints(:, 4));
    title("Raw vs Time");
    xlabel("ROS Time (s)");
    ylabel("Gyro reading (ADC counts)");

    figure
    plot(calPoints(:, 4), calPoints(:, 2), ".");
    title("Reading vs Rate");
    xlabel("Rate (rads/sec)");
    ylabel("Gyro reading (ADC counts)");
end


function idx = findPointClosestToTime(table, sec, nanosec)
    desiredTime = sec + nanosec / 1e9;
    tableTimes = table{:, 1} + table{:, 2} / 1e9;

    absTimeDiffs = abs(tableTimes - desiredTime);
    [~, idx] = min(absTimeDiffs);
end
