% parseCloud.m: Program to extract point cloud from files and plot results
% Author: Nishanth Koganti
% Date: 2015/10/13

function parseCloud(fileName, calibName)

mode = 1;
fid = fopen(fileName);

calibMatrix = load(calibName);

%% plotting initialization
if mode >= 1
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
centers = [];
while ~feof(fid)
    line = fgets(fid);
    dat = str2num(line);
    nPoints = dat(2);
    cloud = zeros(nPoints,3);
    
    for i = 1:nPoints
        line = fgets(fid);
        dat = str2num(line);
        cloud(i,:) = dat;
    end
    
    if mode == 1
        pl1 = plot3(cloud(:,1), cloud(:,2), cloud(:,3), '.r', 'MarkerSize', 10);    
        drawnow;
        pause(0.03);
        delete(pl1);
    end    
end

fclose(fid);

return;