fprintf("Please select input data\n");
[files, bases] = uigetfile("*.csv", "Select data files", "MultiSelect", "on");
fullfiles = fullfile(bases, files);

if ~isa(fullfiles, "cell")
    fullfiles = { fullfiles };
end

if all(contains(fullfiles, "calibration2"))
    fprintf("Running calibration2 analysis\n");
    [fit, gof, filtereddata] = analyzeCalibration2Files(fullfiles);

    rigrates = filtereddata(:, 1);
    gyrorate = filtereddata(:, 2);
    gyrotemp = filtereddata(:, 3);

    %display fit as final act
    fprintf("Final result: \n");
    fit
elseif all(contains(fullfiles, "calibration"))
    fprintf("Running calibration analysis\n");

    [fit, gof] = analyzeCalibrationFiles(fullfiles);

    %display fit as final act
    fprintf("Final result: \n");
    fit
elseif all(contains(fullfiles, "validation"))
    fprintf("Running validation analysis\n");
    analyzeValidationFiles(fullfiles);
elseif all(contains(fullfiles, "recording"))
    fprintf("Running recording analysis\n");
    stepsPerRotation = analyzeRecordingFiles(fullfiles);

    %display final result
    fprintf("Steps per Rotation: %f\n", stepsPerRotation);
else
    error("Could not determine which script to run. Ensure that that" ...
        + " all chosen files have the same type (calibration or validation).");
end
