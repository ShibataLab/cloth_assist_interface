% plotMarker.m: Program to plot motion capture data 
% Author: Nishanth
% Date: 2015/9/19

% TODO:
% 1) Implement in python

function plotMarker(fileName)

printFlag = false;

[~, Data] = parseMocap(sprintf('%s.trc',fileName));

nSamples = size(Data,1);
nCollar = 6;
nLSleeve = 3;
nRSleeve = 3;

close all;
fontSize = 12;
markerSize = 25;
lineWidth = 3;

Data = Data(:,3:end);

fid = figure; 
hold on;
xlabel('X [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
ylabel('Y [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
zlabel('Z [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
title('Mocap Marker Plot',  'FontSize', fontSize, 'FontWeight', 'bold');
set(gca, 'FontSize', fontSize, 'FontWeight', 'bold');
axis([-0.2 1.0 -0.2 0.7 0.5 1.5]);
view([150,30]);

waitforbuttonpress;

for i = 1:nSamples
    collarDat = Data(i,1:nCollar*3);
    collarDat = reshape(collarDat,3,nCollar)';
    collarDat = [collarDat; collarDat(1,:)];
    
    lSleeveDat = Data(i,nCollar*3+1:nCollar*3+nLSleeve*3);
    lSleeveDat = reshape(lSleeveDat,3,nLSleeve)';
    lSleeveDat = [lSleeveDat; lSleeveDat(1,:)];
    
    rSleeveDat = Data(i,nCollar*3+nLSleeve*3+1:nCollar*3+nLSleeve*3+nRSleeve*3);
    rSleeveDat = reshape(rSleeveDat,3,nRSleeve)';
    rSleeveDat = [rSleeveDat; rSleeveDat(1,:)];
    
    pl1 = plot3(collarDat(:,1), collarDat(:,2), collarDat(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
    pl2 = plot3(lSleeveDat(:,1), lSleeveDat(:,2), lSleeveDat(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
    pl3 = plot3(rSleeveDat(:,1), rSleeveDat(:,2), rSleeveDat(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
    drawnow; 
    
    pause(0.001);
    
    if printFlag
        print(fid,sprintf('%s%03d.png',fileName,i),'-dpng');
    end
    
    delete(pl1);
    delete(pl2);
    delete(pl3);
end
hold off;