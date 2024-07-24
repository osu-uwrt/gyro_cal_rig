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
    rigrates = rawvaldata(:, 3);
    expectedvals = rawvaldata(:, 4);
    actualvals = rawvaldata(:, 5);
    positiondiff = expectedvals - actualvals;

    [pdgoodvals, timenopdoutliers, pdnooutliers] = removeoutliers(time, positiondiff);

    pdmaxidx = find(pdnooutliers == max(pdnooutliers));
    pdmaxtime = timenopdoutliers(pdmaxidx);
    pdmaxvalue = pdnooutliers(pdmaxidx);
    
    %drift analysis
    rigrateszero = rigrates(pdgoodvals) == 0;
    pdnooutlierszero = pdnooutliers(rigrateszero);
    timenopdoutlierszero = timenopdoutliers(rigrateszero);

    %difference ratio analysis
    pdexpectedvals = expectedvals(pdgoodvals);
    pdexpectedvalszero = pdexpectedvals(rigrateszero);
    dividableevs = pdexpectedvalszero ~= 0;
    pdratios = pdnooutlierszero(dividableevs) ./ pdexpectedvalszero(dividableevs);
    [~, timenoevoutliers, diffratiosnooutliers] = removeoutliers(timenopdoutlierszero(dividableevs), pdratios);

    plot(timenoevoutliers);

    %final results
    initialdiff = pdnooutlierszero(1);
    finaldiff = pdnooutlierszero(length(pdnooutlierszero));
    totaltime = timenopdoutlierszero(length(timenopdoutlierszero));
    totaldriftrads = finaldiff - initialdiff;
    driftpersdegs = (totaldriftrads * 180 / pi) / totaltime;

    fprintf("Total Time: %f s\n", totaltime);
    fprintf("Initial difference: %f rad\n", initialdiff);
    fprintf("Final difference: %f rad\n", finaldiff);
    fprintf("Drift: %f degs/hr\n", driftpersdegs * 3600);

    %create plots
    fprintf("Creating plots\n");

    figure
    tiledlayout(3, 2);

    nexttile
    plot(rawvaldata(:, 1));
    title("Time");
    ylabel("Time (s)");

    nexttile
    plot(time, rawvaldata(:, 2), '.');
    title("Temperature vs Time");
    xlabel("Time (s)");
    ylabel("Temperature (C)");

    nexttile
    plot(time, rawvaldata(:, 4), '.');
    hold on
    plot(time, rawvaldata(:, 5), '.');
    title("Expected and Actual Position vs Time");
    xlabel("Time (s)");
    ylabel("Position (rad)");
    legend("Expected Position", "Actual Position");

    nexttile
    plot(timenopdoutlierszero, pdnooutlierszero);
    title("Position difference vs Time while not moving");
    xlabel("Time (s)");
    ylabel("Position Difference (rad)");

    nexttile
    plot(timenopdoutliers, pdnooutliers, '.');
    title("Position difference without outliers");
    xlabel("Time (s)");
    ylabel("Position Difference (rad)");
    hold on
    plot(pdmaxtime, pdmaxvalue, "*");

    nexttile
    plot(timenoevoutliers, diffratiosnooutliers, '.');
    title("Error / Expected vs time");
    xlabel("Time (s)");
    ylabel("Error / Expected");
end


function [notoutliers, newtime, newdata] = removeoutliers(time, data)
    m = mean(data);
    sd = std(data);
    
    notoutliers = abs(data - m) < 3 * sd;
    newtime = time(notoutliers);
    newdata = data(notoutliers);
end
