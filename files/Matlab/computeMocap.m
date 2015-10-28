% computeMocap.m: Program to plot motion capture data 
% Author: Nishanth
% Date: 2015/9/19

% TODO:
% 1) Implement in python

function computeMocap(trackName, mocapName, postureName)

% set the flag names
plotFlag = 2;
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

topCoordData = zeros(nSamples,8);
ellipseParamData = zeros(nSamples,19);
markerData = mocapData(mocapInd,3:end);
centeredMarkerData = zeros(nSamples,3*nMarkers);
ellipseMarkerData = zeros(nSamples,(nCollarPoints/4+nSleevePoints/2)*3);

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

% variables for consistent normal estimation
collarNormal = [];
lSleeveNormal = [];
rSleeveNormal = [];
collarMajAxis = [];
collarMinAxis = [];

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
    [collarEllDat, ~, collarMajAxis, collarMinAxis] = ellipseApprox(collarDat,nCollarPoints, collarMajAxis, collarMinAxis);

    lSleeveDat = markerData(i,nCollar*3+1:nCollar*3+nLSleeve*3);
    lSleeveDat = reshape(lSleeveDat,3,nLSleeve)';
    [lSleeveEllDat, ~, lSleeveNormal] = sleeveApprox(lSleeveDat(3,:),bodyData([4 6],:),nSleevePoints, lSleeveNormal);
    
    rSleeveDat = markerData(i,nCollar*3+nLSleeve*3+1:nCollar*3+nLSleeve*3+nRSleeve*3);
    rSleeveDat = reshape(rSleeveDat,3,nRSleeve)';
    [rSleeveEllDat, ~, rSleeveNormal] = sleeveApprox(rSleeveDat(1,:),bodyData([5 7],:),nSleevePoints, rSleeveNormal);
    
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
    title(sprintf('Mocap Marker Plot %s', mocapName),  'FontSize', fontSize, 'FontWeight', 'bold');
    set(gca, 'FontSize', fontSize, 'FontWeight', 'bold');
    axis([-0.5 0.5 -0.5 0.5 -0.5 0.5]);
    view([90,30]);

    % waitforbuttonpress;
end

startCollarInd = [];
startLSleeveInd = [];
startRSleeveInd = [];

for i = 1:nSamples
    collarData = centeredMarkerData(i,1:nCollar*3);
    collarData = reshape(collarData,3,nCollar)';    
    [collarEllData, collarEllParams, collarMajAxis, collarMinAxis] = ellipseApprox(collarData, nCollarPoints/4, collarMajAxis, collarMinAxis);
    
    if isempty(startCollarInd)
        [~,startCollarInd] = min(sqrt(sum((collarEllData - repmat(collarData(1,:),nCollarPoints/4,1)),2).^2));
    end
    collarEllData = circshift(collarEllData,-startCollarInd+1,1);
 
    lSleeveData = centeredMarkerData(i,nCollar*3+1:nCollar*3+nLSleeve*3);
    lSleeveData = reshape(lSleeveData,3,nLSleeve)';
    [lSleeveEllData, lSleeveEllParams, lSleeveNormal] = sleeveCenterApprox(lSleeveData, nSleevePoints/4, lSleeveNormal);
    
    if isempty(startLSleeveInd)
        [~,startLSleeveInd] = min(sqrt(sum((lSleeveEllData - repmat(lSleeveData(1,:),nSleevePoints/4,1)),2).^2));
    end
    lSleeveEllData = circshift(lSleeveEllData,-startLSleeveInd+1,1);
    
    rSleeveData = centeredMarkerData(i,nCollar*3+nLSleeve*3+1:nCollar*3+nLSleeve*3+nRSleeve*3);
    rSleeveData = reshape(rSleeveData,3,nRSleeve)';
    [rSleeveEllData, rSleeveEllParams, rSleeveNormal] = sleeveCenterApprox(rSleeveData, nSleevePoints/4, rSleeveNormal);
    
    if isempty(startRSleeveInd)
        [~,startRSleeveInd] = min(sqrt(sum((rSleeveEllData - repmat(rSleeveData(1,:),nSleevePoints/4,1)),2).^2));
    end
    rSleeveEllData = circshift(rSleeveEllData,-startRSleeveInd+1,1);
    
    cED = reshape(collarEllData',1,3*nCollarPoints/4);
    lSED = reshape(lSleeveEllData',1,3*nSleevePoints/4);
    rSED = reshape(rSleeveEllData',1,3*nSleevePoints/4);
    
    ellipseParamData(i,:) = [collarEllParams lSleeveEllParams rSleeveEllParams];
    ellipseMarkerData(i,:) = [cED lSED rSED];
    
    if plotFlag >= 2
        pl1 = plot3(collarData(:,1), collarData(:,2), collarData(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl2 = plot3(lSleeveData(:,1), lSleeveData(:,2), lSleeveData(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl3 = plot3(rSleeveData(:,1), rSleeveData(:,2), rSleeveData(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl4 = plot3(collarEllData(1,1), collarEllData(1,2), collarEllData(1,3), '.-g', 'MarkerSize', markerSize+10, 'LineWidth', lineWidth);
        pl5 = plot3(lSleeveEllData(1,1), lSleeveEllData(1,2), lSleeveEllData(1,3), '.-g', 'MarkerSize', markerSize+10, 'LineWidth', lineWidth);
        pl6 = plot3(rSleeveEllData(1,1), rSleeveEllData(1,2), rSleeveEllData(1,3), '.-g', 'MarkerSize', markerSize+10, 'LineWidth', lineWidth);
        pl7 = plot3(collarEllData(:,1), collarEllData(:,2), collarEllData(:,3), '.-k', 'MarkerSize', markerSize-5, 'LineWidth', lineWidth);
        pl8 = plot3(lSleeveEllData(:,1), lSleeveEllData(:,2), lSleeveEllData(:,3), '.-k', 'MarkerSize', markerSize-5, 'LineWidth', lineWidth);
        pl9 = plot3(rSleeveEllData(:,1), rSleeveEllData(:,2), rSleeveEllData(:,3), '.-k', 'MarkerSize', markerSize-5, 'LineWidth', lineWidth);
        drawnow; 

        pause(0.001);

        if printFlag
            print(fid1,sprintf('../Results/MocapData/Figures/%s%03d.png',mocapName,i),'-dpng');
        end

        delete(pl1);
        delete(pl2);
        delete(pl3);
        delete(pl4);
        delete(pl5);
        delete(pl6);
        delete(pl7);
        delete(pl8);
        delete(pl9);
    end
end

if plotFlag == 3
    close all;
    xDat = 1:nSamples;
    
    for i = 1:size(topCoordData,2)
        figure;
        plot(xDat,topCoordData(:,i));
    end
end

if plotFlag == 3
    close all;
    xDat = 1:nSamples;
    
    for i = 1:size(ellipseParamData,2)
        figure;
        plot(xDat,ellipseParamData(:,i));
    end
end

% save all the feature files
dlmwrite(sprintf('%sRawMarker',mocapName),markerData);
dlmwrite(sprintf('%sTopCoord',mocapName),topCoordData);
dlmwrite(sprintf('%sMarker',mocapName),centeredMarkerData);
dlmwrite(sprintf('%sEllipseParam',mocapName),ellipseParamData);
dlmwrite(sprintf('%sEllipseMarker',mocapName),ellipseMarkerData);

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
function [outputData, params, majAxis, minAxis] = ellipseApprox(inputData, nPoints, prevMajAxis, prevMinAxis)

nMarkers = size(inputData,1);

nIntpMarkers = nMarkers*5;
inputData = linearApprox(inputData, nIntpMarkers);

center = mean(inputData,1);
dataN = transpose(inputData - repmat(center,nIntpMarkers,1));

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

if prevMajAxis
    if sign(dot(prevMajAxis, majAxis)) == -1
        majAxis = -majAxis;
    end
else
    if sign(sum(majAxis)) == -1
        majAxis = -majAxis;
    end
end

if prevMinAxis
    if sign(dot(prevMinAxis, minAxis)) == -1
        minAxis = -minAxis;
    end
else
    if sign(sum(minAxis)) == -1
        minAxis = -minAxis;
    end
end

theta = linspace(0,pi*2,nPoints+1);
theta = theta(1:end-1);
outputData = transpose(repmat(eCenter,1,nPoints) + majAxisLen*(majAxis*cos(theta)) + minAxisLen*(minAxis*sin(theta)));

params = [eCenter' majAxisLen majAxis' minAxisLen minAxis'];

return

%% Function to approximate sleeve shape
function [outputData, params, normal] = sleeveApprox(inputData, bodyData, nPoints, prevNormal)

wrist = bodyData(1,:);
shoulder = bodyData(2,:);
sleeve = inputData;

minDist = abs(cross(shoulder-wrist,sleeve-wrist))/abs(shoulder-wrist);
nearPoint = (dot(sleeve-shoulder,wrist-shoulder)*wrist+dot(sleeve-wrist,shoulder-wrist)*shoulder)/dot(shoulder-wrist,shoulder-wrist);

normal = (wrist-shoulder)/norm(wrist-shoulder);

if prevNormal
    if sign(dot(normal,prevNormal)) == -1
        normal = -normal;
    end
else
    if sign(sum(normal)) == -1
        normal = -normal;
    end
end

p1 = (sleeve-nearPoint)/norm(sleeve-nearPoint);
p2 = cross(p1,normal);

theta = linspace(0,2*pi,nPoints+1);
theta = theta(1:end-1);
outputData = transpose(repmat(nearPoint',1,nPoints) + minDist*(p1'*sin(theta)) + minDist*(p2'*cos(theta)));

params = [nearPoint minDist];

return

%% Function to approximate centered sleeve shape
function [outputData, params, normal] = sleeveCenterApprox(points, nPoints, prevNormal)

% centering data and computing center
center = mean(points,1);
nMarkers = size(points,1);
radius = mean(sqrt(sum((points - repmat(center,nMarkers,1)).^2,2)));

% computing normal to points
dataN = transpose(points - repmat(center,nMarkers,1));
[U,~,~] = svd(dataN);
normal = U(:,3);

if prevNormal
    if sign(dot(normal,prevNormal)) == -1
        normal = -normal;
    end
else
    if sign(sum(normal)) == -1
        normal = -normal;
    end
end

p1 = (points(1,:)-center)/norm(points(1,:)-center);
p2 = cross(p1,normal);

theta = linspace(0,2*pi,nPoints+1);
theta = theta(1:end-1);
outputData = transpose(repmat(center',1,nPoints) + radius*(p1'*sin(theta)) + radius*(p2'*cos(theta)));

params = [center radius];

return