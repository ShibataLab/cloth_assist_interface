% computeMocap.m: Program to plot motion capture data 
% Author: Nishanth
% Date: 2015/9/19

% TODO:
% 1) Implement in python

function computeMocap(fileName)

plotFlag = false;
printFlag = false;

[~, kinectData] = parseKinect(sprintf('./Tracks/%s',fileName));
[~, mocapData] = parseMocap(sprintf('./MocapData/%s.trc',fileName));

nCollar = 6;
nLSleeve = 3;
nRSleeve = 3;
nMarkers = 12;
nBodyPoints = 20;
nSleevePoints = 20;
nCollarPoints = 40;
nSamples = size(kinectData,1);

offset = 20;
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

topologyCoord = zeros(nSamples,8);
markerData = mocapData(mocapInd,3:end);
centeredMarkerData = zeros(nSamples,3*(nMarkers+1));
[bodyData,head,body,lShoulder,rShoulder] = computePosture(sprintf('./Posture/%s',fileName));

for i = 1:nSamples
    mData = markerData(i,:);
    mData = reshape(mData,3,nMarkers)';
    meanPoint = mean(mData);
    mData = mData - repmat(meanPoint,nMarkers,1);
    mData = [meanPoint; mData];
    mData = reshape(mData',3*(nMarkers+1),1);
    centeredMarkerData(i,:) = mData;
end

if plotFlag
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
    collarData = centeredMarkerData(i,1:nCollar*3);
    collarData = reshape(collarData,3,nCollar)';    
    collarData = ellipseApprox(collarData,nCollarPoints);

    lSleeveDat = markerData(i,nCollar*3+1:nCollar*3+nLSleeve*3);
    lSleeveDat = reshape(lSleeveDat,3,nLSleeve)';
    lSleeveData = centeredMarkerData(i,nCollar*3+1:nCollar*3+nLSleeve*3);
    lSleeveData = reshape(lSleeveDat,3,nLSleeve)';
    lSleeveData = sleeveApprox(lSleeveData(2,:),bodyData([4 6],:),nSleevePoints);

    rSleeveDat = markerData(i,nCollar*3+nLSleeve*3+1:nCollar*3+nLSleeve*3+nRSleeve*3);
    rSleeveDat = reshape(rSleeveDat,3,nRSleeve)';
    rSleeveData = centeredMarkerData(i,nCollar*3+nLSleeve*3+1:nCollar*3+nLSleeve*3+nRSleeve*3);
    rSleeveData = reshape(rSleeveDat,3,nRSleeve)';
    rSleeveData = sleeveApprox(rSleeveData(1,:),bodyData([5 7],:),nSleevePoints);

    topologyCoord(i,:) = topologyCompute(head,body,lShoulder,rShoulder,collarData,lSleeveData,rSleeveData);

    if plotFlag
        pl1 = plot3(collarDat(:,1), collarDat(:,2), collarDat(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl2 = plot3(lSleeveDat(:,1), lSleeveDat(:,2), lSleeveDat(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        pl3 = plot3(rSleeveDat(:,1), rSleeveDat(:,2), rSleeveDat(:,3), '.-m', 'MarkerSize', markerSize, 'LineWidth', lineWidth);
        drawnow; 

        pause(0.001);

        if printFlag
            print(fid,sprintf('../Results/MocapData/Figures/%s%03d.png',fileName,i),'-dpng');
        end

        delete(pl1);
        delete(pl2);
        delete(pl3);
    end
end

dlmwrite(sprintf('../Results/MocapData/%sRawMarker',fileName),markerData);
dlmwrite(sprintf('../Results/MocapData/%sTopCoord',fileName),topologyCoord);
dlmwrite(sprintf('../Results/MocapData/%sMarker',fileName),centeredMarkerData);

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
function outputData = ellipseApprox(inputData, nPoints)

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

return

%% Function to approximate sleeve shape
function outputData = sleeveApprox(inputData, bodyData, nPoints)

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

return