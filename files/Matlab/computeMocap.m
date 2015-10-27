% computeMocap.m: Program to plot motion capture data 
% Author: Nishanth
% Date: 2015/9/19

% TODO:
% 1) Implement in python

function computeMocap(trackName, mocapName, postureName)

% set the flag names
plotFlag = 0;
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
nSamples = size(kinectData,1);

% compute mocap data corresponding to kinect tracks
offset = 15;
mocapT = mocapData(:,2);
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
ellipseData = zeros(nSamples,19);
topCoordData = zeros(nSamples,8);
markerData = mocapData(mocapInd,3:end);
centeredMarkerData = zeros(nSamples,3*nMarkers);

% parse the posture file and get body points
[bodyData,head,body,lShoulder,rShoulder] = computePosture(sprintf('%s',postureName), nBodyPoints);

% compute centered marker data
for i = 1:nSamples
    mData = markerData(i,:);
    mData = reshape(mData,3,nMarkers)';
    meanPoint = mean(mData);
    mData = mData - repmat(meanPoint,nMarkers,1);
    mData = reshape(mData',3*nMarkers,1);
    centeredMarkerData(i,:) = mData;
end

% plot the body posture data
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
    [collarEllDat, ~] = ellipseApprox(collarDat,nCollarPoints);

    lSleeveDat = markerData(i,nCollar*3+1:nCollar*3+nLSleeve*3);
    lSleeveDat = reshape(lSleeveDat,3,nLSleeve)';
    [lSleeveEllDat, ~] = sleeveApprox(lSleeveDat(3,:),bodyData([4 6],:),nSleevePoints);
    
    rSleeveDat = markerData(i,nCollar*3+nLSleeve*3+1:nCollar*3+nLSleeve*3+nRSleeve*3);
    rSleeveDat = reshape(rSleeveDat,3,nRSleeve)';
    [rSleeveEllDat, ~] = sleeveApprox(rSleeveDat(1,:),bodyData([5 7],:),nSleevePoints);
    
    topCoordData(i,:) = topologyCompute(head,body,lShoulder,rShoulder,collarEllDat,lSleeveEllDat,rSleeveEllDat);

    if plotFlag == 3
        pl1 = plot3(collarDat(:,1), collarDat(:,2), collarDat(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl2 = plot3(lSleeveDat(:,1), lSleeveDat(:,2), lSleeveDat(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl3 = plot3(rSleeveDat(:,1), rSleeveDat(:,2), rSleeveDat(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl4 = plot3(collarEllDat(:,1), collarEllDat(:,2), collarEllDat(:,3), '.-g', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl5 = plot3(lSleeveEllDat(:,1), lSleeveEllDat(:,2), lSleeveEllDat(:,3), '.-g', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl6 = plot3(rSleeveEllDat(:,1), rSleeveEllDat(:,2), rSleeveEllDat(:,3), '.-g', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        drawnow; 

        pause(0.033);

        if printFlag
            print(fid,sprintf('../Results/MocapData/Figures/%s%03d.png',mocapName,i),'-dpng');
        end

        delete(pl1);
        delete(pl2);
        delete(pl3);
        delete(pl4);
        delete(pl5);
        delete(pl6);
    end
end

if plotFlag >= 2
    close all;
    fontSize = 12;
    markerSize = 25;
    lineWidth = 3;

    fid1 = figure; 
    hold on;
    xlabel('X [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    ylabel('Y [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    zlabel('Z [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    title('Mocap Marker Plot',  'FontSize', fontSize, 'FontWeight', 'bold');
    set(gca, 'FontSize', fontSize, 'FontWeight', 'bold');
    axis([-0.5 0.5 -0.5 0.5 -0.5 0.5]);
    view([150,30]);

    waitforbuttonpress;
end

for i = 1:nSamples
    collarData = centeredMarkerData(i,1:nCollar*3);
    collarData = reshape(collarData,3,nCollar)';    
    [collarEllData, collarEllParams] = ellipseApprox(collarData,nCollarPoints);

    lSleeveData = centeredMarkerData(i,nCollar*3+1:nCollar*3+nLSleeve*3);
    lSleeveData = reshape(lSleeveData,3,nLSleeve)';
    [lSleeveEllData, lSleeveEllParams] = sleeveCenterApprox(lSleeveData,nSleevePoints);

    rSleeveData = centeredMarkerData(i,nCollar*3+nLSleeve*3+1:nCollar*3+nLSleeve*3+nRSleeve*3);
    rSleeveData = reshape(rSleeveData,3,nRSleeve)';
    [rSleeveEllData, rSleeveEllParams] = sleeveCenterApprox(rSleeveData,nSleevePoints);

    ellipseData(i,:) = [collarEllParams lSleeveEllParams rSleeveEllParams];

    if plotFlag >= 2
        pl1 = plot3(collarData(:,1), collarData(:,2), collarData(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl2 = plot3(lSleeveData(:,1), lSleeveData(:,2), lSleeveData(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl3 = plot3(rSleeveData(:,1), rSleeveData(:,2), rSleeveData(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl4 = plot3(collarEllData(:,1), collarEllData(:,2), collarEllData(:,3), '.-g', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl5 = plot3(lSleeveEllData(:,1), lSleeveEllData(:,2), lSleeveEllData(:,3), '.-g', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl6 = plot3(rSleeveEllData(:,1), rSleeveEllData(:,2), rSleeveEllData(:,3), '.-g', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        drawnow; 

        pause(0.033);

        if printFlag
            print(fid1,sprintf('../Results/MocapData/Figures/%s%03d.png',mocapName,i),'-dpng');
        end

        delete(pl1);
        delete(pl2);
        delete(pl3);
        delete(pl4);
        delete(pl5);
        delete(pl6);
    end
end

if plotFlag >= 1
    close all;
    xDat = 1:nSamples;
    
    for i = 1:size(topCoordData,2)
        figure;
        plot(xDat,topCoordData(:,i));
    end
end

% save all the feature files
dlmwrite(sprintf('%sEllipse',mocapName),ellipseData);
dlmwrite(sprintf('%sRawMarker',mocapName),markerData);
dlmwrite(sprintf('%sTopCoord',mocapName),topCoordData);
dlmwrite(sprintf('%sMarker',mocapName),centeredMarkerData);

%% Function for piecewise linear approximation
function outputData = linearApprox(inputData, nPoints)

inputData = [inputData; inputData(1,:)];
nMarkers = size(inputData,1);

% Linear Approximation
limits = 1:nMarkers;
fractions = linspace(1,nMarkers,nPoints+1);
fractions = fractions(1:end-1);

outputData = interp1(limits,inputData,fractions);
return

%% Function for ellipse approximation
function [outputData, params] = ellipseApprox(inputData, nPoints)

nMarkers = size(inputData,1);

center = mean(inputData,1);
dataN = transpose(inputData - repmat(center,nMarkers,1));

% computing normal to points
[U,~,~] = svd(dataN);
normal = U(:,3);

tmatrix = [1 - normal(1)^2/(1+normal(3))      -normal(1)*normal(2)/(1+normal(3)) -normal(1);
           -normal(1)*normal(2)/(1+normal(3)) 1 - normal(2)^2/(1+normal(3))      -normal(2);
           normal(1)                          normal(2)                          normal(3)];       
    
data = transpose(tmatrix*dataN);
params = ellipseFit(data(:,1),data(:,2));

eCenter = [params(1); params(2); 0];
eCenter = tmatrix'*eCenter + center';

if params(3) > params(4)
    majAxisLen = params(3);    
    majAxis = [cos(params(5)); sin(params(5)); 0];
    minAxisLen = params(4);
    minAxis = [-sin(params(5)); cos(params(5)); 0];
else
    majAxisLen = params(4);    
    majAxis = [-sin(params(5)); cos(params(5)); 0];
    minAxisLen = params(3);
    minAxis = [cos(params(5)); sin(params(5)); 0];
end

majAxis = tmatrix'*majAxis;
minAxis = tmatrix'*minAxis;

theta = linspace(0,pi*2,nPoints+1);
theta = theta(1:end-1);
outputData = transpose(repmat(eCenter,1,nPoints) + majAxisLen*(majAxis*cos(theta)) + minAxisLen*(minAxis*sin(theta)));

params = [eCenter' majAxisLen majAxis' minAxisLen minAxis'];

return

%% Function to approximate sleeve shape
function [outputData, params] = sleeveApprox(inputData, bodyData, nPoints)

wrist = bodyData(1,:);
shoulder = bodyData(2,:);
sleeve = inputData;

minDist = abs(cross(shoulder-wrist,sleeve-wrist))/abs(shoulder-wrist);
nearPoint = (dot(sleeve-shoulder,wrist-shoulder)*wrist+dot(sleeve-wrist,shoulder-wrist)*shoulder)/dot(shoulder-wrist,shoulder-wrist);

normal = (wrist-shoulder)/norm(wrist-shoulder);

p1 = (sleeve-nearPoint)/norm(sleeve-nearPoint);
p2 = cross(p1,normal);

theta = linspace(0,2*pi,nPoints+1);
theta = theta(1:end-1);
outputData = transpose(repmat(nearPoint',1,nPoints) + minDist*(p1'*sin(theta)) + minDist*(p2'*cos(theta)));

params = [nearPoint minDist];

return

%% Function to approximate centered sleeve shape
function [outputData, params] = sleeveCenterApprox(points, nPoints)

% centering data and computing center
center = mean(points,1);
nMarkers = size(points,1);
radius = mean(sqrt(sum((points - repmat(center,nMarkers,1)).^2,2)));

% computing normal to points
dataN = transpose(points - repmat(center,nMarkers,1));
[U,~,~] = svd(dataN);
normal = U(:,3);

p1 = (points(1,:)-center)/norm(points(1,:)-center);
p2 = cross(p1,normal);

theta = linspace(0,2*pi,nPoints+1);
theta = theta(1:end-1);
outputData = transpose(repmat(center',1,nPoints) + radius*(p1'*sin(theta)) + radius*(p2'*cos(theta)));

params = [center radius];

return