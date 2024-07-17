function [fitresult, gof] = fitcalibrationdata(filteredgyrorates, filteredtemps, filteredrigrates)
%CREATEFIT(FILTEREDGYRORATES,FILTEREDTEMPS,FILTEREDRIGRATES)
%  Create a fit.
%
%  Data for 'untitled fit 1' fit:
%      X Input: filteredgyrorates
%      Y Input: filteredtemps
%      Z Output: filteredrigrates
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  See also FIT, CFIT, SFIT.

%  Auto-generated by MATLAB on 16-Jul-2024 00:31:03


%% Fit: 'untitled fit 1'.
% [xData, yData, zData] = prepareSurfaceData( filteredgyrorates, filteredtemps, filteredrigrates );
[filteredgyrorates, filteredtemps, filteredrigrates] = prepareSurfaceData( filteredgyrorates, filteredtemps, filteredrigrates );

% Set up fittype and options.
ft = fittype( 'poly21' );

% Fit model to data.
[fitresult, gof] = fit( [filteredgyrorates, filteredtemps], filteredrigrates, ft, 'Normalize', 'on' );

% Plot fit with data.
figure( 'Name', 'Gyro calibration surface' );
h = plot( fitresult, [filteredgyrorates, filteredtemps], filteredrigrates );
legend( h, 'Gyro calibration surface', 'filteredrigrates vs. filteredgyrorates, filteredtemps', 'Location', 'NorthEast', 'Interpreter', 'none' );
% Label axes
xlabel( 'filteredgyrorates', 'Interpreter', 'none' );
ylabel( 'filteredtemps', 'Interpreter', 'none' );
zlabel( 'filteredrigrates', 'Interpreter', 'none' );
grid on
view( 12.0, 4.3 );


