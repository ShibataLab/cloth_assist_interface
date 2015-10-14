% parseHist.m: Program to extract histogram from files and plot results
% Author: Nishanth Koganti
% Date: 2015/10/13

function Data = parseHist(fileName)

mode = 1;

% getting the data
Data = load(fileName);
Data = Data(:, 2:end);

if mode == 0
    return;
end

nDims = size(Data,2);
nSamples = size(Data,1);

xData = 1:nDims;


%% Plotting
fontSize = 15;

figure;

hold on;
title('Histogram Plotter', 'FontSize', fontSize, 'FontWeight', 'bold');
xlabel('Descriptor', 'FontSize', fontSize, 'FontWeight', 'bold');
ylabel('Value', 'FontSize', fontSize, 'FontWeight', 'bold');
set(gca, 'FontSize', fontSize, 'FontWeight', 'bold');

waitforbuttonpress;

for i = 1:nSamples
    pl = plot(xData, Data(i,:), '-b', 'LineWidth', 2);
    
    drawnow;
    pause(0.03);
    delete(pl);
end
hold off;

return;