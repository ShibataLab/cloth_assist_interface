% computeMocap.m: Program to plot motion capture data 
% Author: Nishanth
% Date: 2015/11/02

% TODO:
% 1) Implement in python

function computeMocap(trackName, mocapName, postureName)

% set the flag names
plotFlag = 1;
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
circleParamData = zeros(nSamples,21);
markerData = mocapData(mocapInd,3:end);
centeredMarkerData = zeros(nSamples,3*nMarkers);
circleMarkerData = zeros(nSamples,(nCollarPoints*markerDownfactor+nSleevePoints*2*markerDownfactor)*3);

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
    [collarCircDat, ~, collarNormal] = circleApprox(collarDat,nCollarPoints, collarNormal, 1);

    lSleeveDat = markerData(i,nCollar*3+1:nCollar*3+nLSleeve*3);
    lSleeveDat = reshape(lSleeveDat,3,nLSleeve)';
    [lSleeveCircDat, lSleeveParams, lSleeveNormal] = sleeveApprox(lSleeveDat,bodyData([4 6],:),nSleevePoints,lSleeveNormal);
    
    rSleeveDat = markerData(i,nCollar*3+nLSleeve*3+1:nCollar*3+nLSleeve*3+nRSleeve*3);
    rSleeveDat = reshape(rSleeveDat,3,nRSleeve)';
    [rSleeveCircDat, rSleeveParams, rSleeveNormal] = sleeveApprox(rSleeveDat,bodyData([5 7],:),nSleevePoints, rSleeveNormal);
    
    topCoordData(i,:) = topologyCompute(head,body,lShoulder,rShoulder,collarCircDat,lSleeveCircDat,rSleeveCircDat);

    if plotFlag == 3
        pl1 = plot3(collarDat(:,1), collarDat(:,2), collarDat(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl2 = plot3(lSleeveDat(:,1), lSleeveDat(:,2), lSleeveDat(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl3 = plot3(rSleeveDat(:,1), rSleeveDat(:,2), rSleeveDat(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl4 = plot3(collarCircDat(:,1), collarCircDat(:,2), collarCircDat(:,3), '.-g', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl5 = plot3(lSleeveCircDat(:,1), lSleeveCircDat(:,2), lSleeveCircDat(:,3), '.-g', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl6 = plot3(rSleeveCircDat(:,1), rSleeveCircDat(:,2), rSleeveCircDat(:,3), '.-g', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl7 = plot3([lSleeveParams(1); rSleeveParams(1)], [lSleeveParams(2); rSleeveParams(2)], [lSleeveParams(3); rSleeveParams(3)], '.m', 'MarkerSize', markerSize+5, 'LineWidth', lineWidth);
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
        delete(pl7);
    end
end

if plotFlag >= 2
    close all;
    fontSize = 12;
    lineWidth = 3;
    markerSize = 25;

    fid1 = figure; 
    hold on;
    xlabel('X [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    ylabel('Y [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    zlabel('Z [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    title(sprintf('Mocap Marker Plot %s', mocapName),  'FontSize', fontSize, 'FontWeight', 'bold');
    set(gca, 'FontSize', fontSize, 'FontWeight', 'bold');
    axis([-0.5 0.5 -0.5 0.5 -0.5 0.5]);
    view([90,30]);

    waitforbuttonpress;
end

for i = 1:nSamples
    collarData = centeredMarkerData(i,1:nCollar*3);
    collarData = reshape(collarData,3,nCollar)';    
    [collarCircData, collarCircParams, collarNormal] = circleApprox(collarData, nCollarPoints*markerDownfactor, collarNormal, 1);
     
    lSleeveData = centeredMarkerData(i,nCollar*3+1:nCollar*3+nLSleeve*3);
    lSleeveData = reshape(lSleeveData,3,nLSleeve)';
    [lSleeveCircData, lSleeveCircParams, lSleeveNormal] = circleApprox(lSleeveData, nSleevePoints*markerDownfactor, lSleeveNormal, 3);
    
    rSleeveData = centeredMarkerData(i,nCollar*3+nLSleeve*3+1:nCollar*3+nLSleeve*3+nRSleeve*3);
    rSleeveData = reshape(rSleeveData,3,nRSleeve)';
    [rSleeveCircData, rSleeveCircParams, rSleeveNormal] = circleApprox(rSleeveData, nSleevePoints*markerDownfactor, rSleeveNormal, 1);
    
    cED = reshape(collarCircData',1,3*nCollarPoints*markerDownfactor);
    lSED = reshape(lSleeveCircData',1,3*nSleevePoints*markerDownfactor);
    rSED = reshape(rSleeveCircData',1,3*nSleevePoints*markerDownfactor);
    
    circleParamData(i,:) = [collarCircParams lSleeveCircParams rSleeveCircParams];
    circleMarkerData(i,:) = [cED lSED rSED];
    
    if plotFlag >= 2
        pl1 = plot3(collarData(:,1), collarData(:,2), collarData(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl2 = plot3(lSleeveData(:,1), lSleeveData(:,2), lSleeveData(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl3 = plot3(rSleeveData(:,1), rSleeveData(:,2), rSleeveData(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl4 = plot3(collarCircData(1,1), collarCircData(1,2), collarCircData(1,3), '.-g', 'MarkerSize', markerSize+10, 'LineWidth', lineWidth);
        pl5 = plot3(lSleeveCircData(1,1), lSleeveCircData(1,2), lSleeveCircData(1,3), '.-g', 'MarkerSize', markerSize+10, 'LineWidth', lineWidth);
        pl6 = plot3(rSleeveCircData(1,1), rSleeveCircData(1,2), rSleeveCircData(1,3), '.-g', 'MarkerSize', markerSize+10, 'LineWidth', lineWidth);
        pl7 = plot3(collarCircData(:,1), collarCircData(:,2), collarCircData(:,3), '.-k', 'MarkerSize', markerSize-5, 'LineWidth', lineWidth);
        pl8 = plot3(lSleeveCircData(:,1), lSleeveCircData(:,2), lSleeveCircData(:,3), '.-k', 'MarkerSize', markerSize-5, 'LineWidth', lineWidth);
        pl9 = plot3(rSleeveCircData(:,1), rSleeveCircData(:,2), rSleeveCircData(:,3), '.-k', 'MarkerSize', markerSize-5, 'LineWidth', lineWidth);
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
    
    waitforbuttonpress;
end

if plotFlag == 3
    close all;
    xDat = 1:nSamples;
    
    for i = 1:size(circleParamData,2)
        figure;
        plot(xDat,circleParamData(:,i));
    end
end

% save all the feature files
dlmwrite(sprintf('%sRawMarker',mocapName),markerData);
dlmwrite(sprintf('%sTopCoord',mocapName),topCoordData);
dlmwrite(sprintf('%sMarker',mocapName),centeredMarkerData);
dlmwrite(sprintf('%sCircleParam',mocapName),circleParamData);
dlmwrite(sprintf('%sCircleMarker',mocapName),circleMarkerData);

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
function [outputData, params, majAxis, minAxis] = collarEllipseApprox(inputData, nPoints, prevMajAxis, prevMinAxis)

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
nMarkers = size(inputData,1);

radii = zeros(nMarkers,1);
centers = zeros(nMarkers,3);

for i = 1:nMarkers,
    sleeve = inputData(i,:);
    radii(i) = abs(cross(shoulder-wrist,sleeve-wrist))/abs(shoulder-wrist);
    centers(i,:) = (dot(sleeve-shoulder,wrist-shoulder)*wrist+dot(sleeve-wrist,shoulder-wrist)*shoulder)/dot(shoulder-wrist,shoulder-wrist);
end

radius = 0.06;
center = mean(centers,1);
normal = (wrist-shoulder)/norm(wrist-shoulder);

axis1 = (inputData(1,:)-center)/norm(inputData(1,:)-center);
axis2 = cross(axis1,normal);

theta = linspace(0,2*pi,nPoints+1);
theta = theta(1:end-1);
outputData = transpose(repmat(center',1,nPoints) + radius*(axis1'*sin(theta)) + radius*(axis2'*cos(theta)));

params = [center normal radius];

return

%% Function to approximate centered sleeve shape
function [outputData, params, normal] = circleApprox(inputData, nPoints, prevNormal, startIndex)

% centering data and computing center
center = mean(inputData,1);
nMarkers = size(inputData,1);
radius = mean(sqrt(sum((inputData - repmat(center,nMarkers,1)).^2,2)));

% computing normal to points
dataN = transpose(inputData - repmat(center,nMarkers,1));
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

axis1 = (inputData(startIndex,:)-center)/norm(inputData(startIndex,:)-center);
axis2 = cross(axis1,normal');

theta = linspace(0,2*pi,nPoints+1);
theta = theta(1:end-1);
outputData = transpose(repmat(center',1,nPoints) + radius*(axis2'*sin(theta)) + radius*(axis1'*cos(theta)));

params = [center normal' radius];

return