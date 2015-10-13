% parseHist.m: Program to extract histogram from files and plot results
% Author: Nishanth Koganti
% Date: 2015/10/13

function Data = parseHist(fileName)

mode = 1;
fid = fopen(fileName);

% Getting the Data
Data = [];
while ~feof(fid)
      line = fgets(fid);
      dat = str2num(line);
      Data = [Data; dat(2:end)];
end

fclose(fid);

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

for i = 1:nSamples
    pl = plot(xData, Data(i,:), '-b', 'LineWidth', 2);
    
    drawnow;
    pause(0.03);
    delete(pl);
end
hold off;

return;