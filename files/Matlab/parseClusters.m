% parseClusters.m: Program to extract cluster centers from files and plot results
% Author: Nishanth Koganti
% Date: 2015/10/13

function parseClusters(fileName)

mode = 1;
fid = fopen(fileName);

%% plotting initialization
if mode == 2
    fontSize = 15;

    figure;
    hold on;
    title('Cloud Plotter', 'FontSize', fontSize, 'FontWeight', 'bold');
    xlabel('X', 'FontSize', fontSize, 'FontWeight', 'bold');
    ylabel('Y', 'FontSize', fontSize, 'FontWeight', 'bold');
    zlabel('Z', 'FontSize', fontSize, 'FontWeight', 'bold');
    set(gca, 'FontSize', fontSize, 'FontWeight', 'bold');
end

%% parsing data
clusterData = [];

while ~feof(fid)
    line = fgets(fid);
    dat = str2num(line);
    nPoints = dat(2);
    clusters = zeros(nPoints,3);
    
    for i = 1:nPoints
        line = fgets(fid);
        dat = str2num(line);
        clusters(i,:) = dat;
    end
    
    
    
    if mode == 2
        pl1 = plot3(clusters(:,1), clusters(:,2), clusters(:,3), '.r', 'MarkerSize', 10);    
        drawnow;
        pause(0.03);
        delete(pl1);
    end    
end

fclose(fid);

return;