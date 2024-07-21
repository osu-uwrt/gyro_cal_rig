fprintf("Please select input data\n");
[files, bases] = uigetfile("*.csv", "Select data files", "MultiSelect", "on");
fullfiles = fullfile(bases, files);

if ~isa(fullfiles, "cell")
    fullfiles = { fullfiles };
end

[fit, gof] = analyzeCalibrationFiles(fullfiles);

%display fit as final act
fprintf("Final result: \n");
fit
