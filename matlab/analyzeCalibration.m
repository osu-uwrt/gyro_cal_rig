fprintf("Please select input data\n");
[files, bases] = uigetfile("*.csv", "Select data files", "MultiSelect", "on");
fullfiles = fullfile(bases, files);

[fit, gof] = analyzeCalibrationFiles(fullfiles);

%display fit as final act
fprintf("Final result: \n");
fit
