function analyzeValidationFiles(files)
    format compact
    
    fprintf("Populating arrays\n");
    
    rawvaldata = []; % order: times, temps, rigrates, expectedyaw, actualyaw
    
    for i = 1 : length(files)
        filename = files{i};
        fprintf("Loading table %d/%d (%s)\n", i, length(files), filename);
        table = readtable(filename, "NumHeaderLines", 1);
    
        tabletimes = table{:, 1} + (table{:, 2} / 1000000000) - table{1, 1};
        tablegyrotemps = table{:, 3};
        tablerigrates = table{:, 4};
        tableexpectedyaw = table{:, 8};
        tableactualyaw = table{:, 9};

        rawvaldata = [rawvaldata; tabletimes, tablegyrotemps, ...
            tablerigrates, tableexpectedyaw, tableactualyaw];
    end

    %process data
    fprintf("Processing data\n");
    
    %negate expected data because fog is upside down in rig
    rawvaldata(:, 4) = -1 * rawvaldata(:, 4);
    rawvaldata(:, 5) = unwrap(rawvaldata(:, 5));

    time = rawvaldata(:, 1);
    expectedvals = rawvaldata(:, 4);

    positiondiff = rawvaldata(:, 4) - rawvaldata(:, 5);
    pdmean = mean(positiondiff);
    pdstd = std(positiondiff);
    
    notoutliers = abs(positiondiff - pdmean) < 3 * pdstd;
    timenooutliers = time(notoutliers);
    pdnooutliers = positiondiff(notoutliers);
    evnooutliers = expectedvals(notoutliers);

    pdmaxidx = find(pdnooutliers == max(pdnooutliers));
    pdmaxtime = timenooutliers(pdmaxidx);
    pdmaxvalue = pdnooutliers(pdmaxidx);
    
    diffratiosnooutliers = evnooutliers ./ pdnooutliers;

    %create plots
    fprintf("Creating plots\n");

    figure
    tiledlayout(3, 2);

    nexttile
    plot(rawvaldata(:, 1));
    title("Time");
    ylabel("Time (s)");

    nexttile
    plot(time, rawvaldata(:, 2));
    title("Temperature vs Time");
    xlabel("Time (s)");
    ylabel("Temperature (C)");

    nexttile
    plot(time, rawvaldata(:, 4));
    hold on
    plot(time, rawvaldata(:, 5));
    title("Expected and Actual Position vs Time");
    xlabel("Time (s)");
    ylabel("Position (rad)");
    legend("Expected Position", "Actual Position");

    nexttile
    plot(time, positiondiff);
    title("Position difference vs Time");
    xlabel("Time (s)");
    ylabel("Position Difference (rad)");

    nexttile
    plot(timenooutliers, pdnooutliers);
    title("Position difference without outliers");
    xlabel("Time (s)");
    ylabel("Position Difference (rad)");
    hold on
    plot(pdmaxtime, pdmaxvalue, "*");

    nexttile
    plot(timenooutliers, diffratiosnooutliers);
    title("Expected / Error vs time");
    xlabel("Time (s)");
    ylabel("Expected / Error");

    %display final results
    fprintf("Maximum difference: %f\n", pdmaxvalue);
    fprintf("Maximum error ratio: %f\n", diffratiosnooutliers(pdmaxidx))
end
