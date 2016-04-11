% inspectMocap.m: Program to plot motion capture data and inspect
% Author: Nishanth
% Date: 2015/11/06

% TODO:
% 1) Implement in python

function inspectMocap2(mocapName, postureName)

% set the flag names
plotFlag = 3;
printFlag = true;

% parse the track file and mocap data
mocapData = load(sprintf('%s',mocapName));

% parameter initialization
nCollar = 6;
nLSleeve = 3;
nRSleeve = 3;
nMarkers = 12;
nBodyPoints = 20;
nSleevePoints = 20;
nCollarPoints = 40;
markerDownfactor = 1/4;
nSamples = size(mocapData,1);

% variables for features
markerData = mocapData;

% parse the posture file and get body points
[bodyData,~,~,~,~] = computePosture(sprintf('%s',postureName), nBodyPoints);

% variables for consistent normal estimation
collarNormal = [];
lSleeveNormal = [];
rSleeveNormal = [];

% % plot the body posture data
if plotFlag == 3
    close all;
    fontSize = 18;
    markerSize = 30;
    lineWidth = 5;

    fid = figure; 
    hold on;
    %xlabel('X [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    %ylabel('Y [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    %zlabel('Z [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    %title('Mocap Marker Plot',  'FontSize', fontSize, 'FontWeight', 'bold');
    set(gca,'xtick',[])
    set(gca,'ytick',[])
    set(gca,'ztick',[])
    set(gca, 'FontSize', fontSize, 'FontWeight', 'bold');
    axis([0.1 0.8 0.0 0.65 0.7 1.5]);
    view([75,10]);

    plot3(bodyData(:,1), bodyData(:,2), bodyData(:,3), '.r', 'MarkerSize', markerSize);
    plot3(bodyData(1:3,1), bodyData(1:3,2), bodyData(1:3,3), '-b', 'LineWidth', lineWidth);
    plot3(bodyData([4,2,5],1), bodyData([4,2,5],2), bodyData([4,2,5],3), '-b', 'LineWidth', lineWidth);
    plot3(bodyData([4,6],1), bodyData([4,6],2), bodyData([4,6],3), '-b', 'LineWidth', lineWidth);
    plot3(bodyData([5,7],1), bodyData([5,7],2), bodyData([5,7],3), '-b', 'LineWidth', lineWidth);
    
    waitforbuttonpress;
end

for i = 1:nSamples
    collarDat = markerData(i,1:nCollar*3);
    collarDat = reshape(collarDat,3,nCollar)';
    collarDat = [collarDat; collarDat(1,:)];

    lSleeveDat = markerData(i,nCollar*3+1:nCollar*3+nLSleeve*3);
    lSleeveDat = reshape(lSleeveDat,3,nLSleeve)';
    lSleeveDat = [lSleeveDat; lSleeveDat(1,:)];
    
    rSleeveDat = markerData(i,nCollar*3+nLSleeve*3+1:nCollar*3+nLSleeve*3+nRSleeve*3);
    rSleeveDat = reshape(rSleeveDat,3,nRSleeve)';
    rSleeveDat = [rSleeveDat; rSleeveDat(1,:)];
    
    if plotFlag == 3
        pl1 = plot3(collarDat(:,1), collarDat(:,2), collarDat(:,3), '.-', 'Color', [0 0.5 0], 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl2 = plot3(lSleeveDat(:,1), lSleeveDat(:,2), lSleeveDat(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl3 = plot3(rSleeveDat(:,1), rSleeveDat(:,2), rSleeveDat(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        legend([pl1,pl2],'Collar','Sleeve','Location','NorthEast');
        set(gcf,'Name',sprintf('Frame: %d',i));
        drawnow; 

        pause(0.1);

        if printFlag
            print(fid,sprintf('./mocap%03d.png',i),'-dpng');
        end

        delete(pl1);
        delete(pl2);
        delete(pl3);
    end
end