% inspectMocap.m: Program to plot motion capture data and inspect
% Author: Nishanth
% Date: 2015/11/06

% TODO:
% 1) Implement in python

function inspectMocap(trackName, mocapName, postureName, startTime)

% set the flag names
plotFlag = 3;
printFlag = false;

% parse the track file and mocap data
kinectData = parseKinectTracks(sprintf('%s',trackName));
[~, mocapData] = parseMocap(sprintf('%s.trc',mocapName));

% parameter initialization
nCollar = 6;
nLSleeve = 3;
nRSleeve = 3;
nMarkers = 12;
nBodyPoints = 20;
nSleevePoints = 20;
nCollarPoints = 40;
markerDownfactor = 1/4;
nSamples = size(kinectData,1);

% compute mocap data corresponding to kinect tracks
offset = 0;
mocapT = mocapData(:,2) + startTime;
kinectT = kinectData(:,2);
mocapInd = zeros(nSamples,1);

for i = 1:nSamples
    tRef = kinectT(i);
    [~,ind] = min((tRef - mocapT).^2);
    if ind - offset < 1
        mocapInd(i) = 1;
    else
        mocapInd(i) = ind - offset;
    end
end

% variables for features
markerData = mocapData(mocapInd,3:end);
centeredMarkerData = zeros(nSamples,3*nMarkers);

% parse the posture file and get body points
[bodyData,~,~,~,~] = computePosture(sprintf('%s',postureName), nBodyPoints);

% compute centered marker data
for i = 1:nSamples
    mData = markerData(i,:);
    mData = reshape(mData,3,nMarkers)';
    meanPoint = mean(mData);
    mData = mData - repmat(meanPoint,nMarkers,1);
    mData = reshape(mData',3*nMarkers,1);
    centeredMarkerData(i,:) = mData;
end

% variables for consistent normal estimation
collarNormal = [];
lSleeveNormal = [];
rSleeveNormal = [];

% % plot the body posture data
if plotFlag == 3
    close all;
    fontSize = 12;
    markerSize = 25;
    lineWidth = 3;

    fid = figure; 
    hold on;
    xlabel('X [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    ylabel('Y [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    zlabel('Z [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    title('Mocap Marker Plot',  'FontSize', fontSize, 'FontWeight', 'bold');
    set(gca, 'FontSize', fontSize, 'FontWeight', 'bold');
    axis([-0.2 1.0 -0.2 0.7 0.5 1.5]);
    view([150,30]);

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

    lSleeveDat = markerData(i,nCollar*3+1:nCollar*3+nLSleeve*3);
    lSleeveDat = reshape(lSleeveDat,3,nLSleeve)';
    
    rSleeveDat = markerData(i,nCollar*3+nLSleeve*3+1:nCollar*3+nLSleeve*3+nRSleeve*3);
    rSleeveDat = reshape(rSleeveDat,3,nRSleeve)';
    
    if plotFlag == 3
        pl1 = plot3(collarDat(:,1), collarDat(:,2), collarDat(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl2 = plot3(lSleeveDat(:,1), lSleeveDat(:,2), lSleeveDat(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl3 = plot3(rSleeveDat(:,1), rSleeveDat(:,2), rSleeveDat(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        set(gcf,'Name',sprintf('Frame: %d',i));
        drawnow; 

        pause();

        if printFlag
            print(fid,sprintf('../Results/MocapData/Figures/%s%03d.png',mocapName,i),'-dpng');
        end

        delete(pl1);
        delete(pl2);
        delete(pl3);
    end
end