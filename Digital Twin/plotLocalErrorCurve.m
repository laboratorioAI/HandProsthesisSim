function [hFig, hAx] = plotLocalErrorCurve(timeVec, errVec, labelText, titleText)
    % plotLocalErrorCurve  Generic plot of a single local error curve on semilog-y axes.
    %
    %   [hFig, hAx] = plotLocalErrorCurve(timeVec, errVec, labelText, titleText)
    %   creates a standalone figure and plots errVec vs timeVec using semilog-y.
    %
    % INPUT
    %   timeVec   : (n×1) double vector of timestamps (e.g. obj.time(2:end)')
    %   errVec    : (n×1) double vector of local errors
    %   labelText : string or char label for Y-axis (e.g. '\theta_2')
    %   titleText : string or char figure title
    %
    % OUTPUT
    %   hFig : figure handle
    %   hAx  : axes handle
    %
    % EXAMPLE:
    %   plotLocalErrorCurve(obj.time(2:end), obj.err2, '\theta_2', 'Middle phalanx error');
    % -------------------------------------------------------------------------
    
    arguments
        timeVec   (:,1) double
        errVec    (:,1) double
        labelText (1,:) char     % or string, for y-axis
        titleText (1,:) char     % or string, for title
    end
    
    % Create new figure
    hFig = figure( ...
        'Name',        ['Local error ', labelText], ...
        'NumberTitle', 'off', ...
        'Color',       'w', ...
        'Units',       'pixels', ...
        'Position',    [100 100 800 550]);
    
    % Create axes
    hAx = axes('Parent', hFig, ...
               'Box',      'on', ...
               'FontSize', 11, ...
               'XScale',   'linear', ...
               'YScale',   'log', ...
               'NextPlot', 'add');
    
    % Plot the curve
    semilogy(hAx, timeVec, errVec, 'LineWidth', 1.4);
    
    % Labeling and formatting
    grid(hAx, 'on');
    xlabel(hAx, 'time  [s]');
    ylabel(hAx, ['local error  ', labelText, '  (England 2–step)']);
    title(hAx, titleText);
    axis(hAx, 'tight');
end