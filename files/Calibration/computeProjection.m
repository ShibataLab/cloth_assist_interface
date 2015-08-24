% computeProjection.m: Program to spatially align Optitrack and Kinect Readings
% and apply absolute orientation algorithm for calibration
% Author: Nishanth Koganti
% Date: 2015/8/22

% TODO:
% 1) Implement in python
% 2) Find offset automatically

function computeProjection(fileName, offset, mode)

close all;
fontSize = 12;
markerSize = 25;

if mode == 0
    % Kinect Calibration Mode
    [~,kinectData] = parseKinect(fileName);
    [~,mocapData] = parseMocap(sprintf('%s.trc',fileName));

    %nMocap = size(mocapData,1);
    nKinect = size(kinectData,1);
    nMarkers = (size(mocapData,2) - 2)/3;

   mocapT = mocapData(:,2);
   kinectT = kinectData(:,2);
   mocapInd = zeros(nKinect,1);
    for j = 1:nKinect
        tRef = kinectT(j);
        [~,ind] = min((tRef - mocapT).^2);
        if ind - offset < 1
            mocapInd(j) = 1;
        else
            mocapInd(j) = ind - offset;
        end
    end

    kinPos = kinectData(:,3:5);
    mocapPos = zeros(nKinect,3);

    for j = 1:nMarkers
        mocapPos(:,1) = mocapPos(:,1) + mocapData(mocapInd,3+(j-1)*3);
        mocapPos(:,2) = mocapPos(:,2) + mocapData(mocapInd,4+(j-1)*3);
        mocapPos(:,3) = mocapPos(:,3) + mocapData(mocapInd,5+(j-1)*3);
    end
    mocapPos = mocapPos./nMarkers;

    [R,T,c,err,kOut] = absoluteOrientationSVD(kinPos',mocapPos');
    kinOut = kOut';
    transMatrix = [c*R; 0 0 0];
    transMatrix = [transMatrix [T; 1]];
    dlmwrite('KinectCalibration', transMatrix);
    fprintf('Error: %f\n',err);

    figure;
    hold on;
    xlabel('X [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    ylabel('Y [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    zlabel('Z [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    title('Kinect Calibration',  'FontSize', fontSize, 'FontWeight', 'bold');
    set(gca, 'FontSize', fontSize, 'FontWeight', 'bold');
    axis([-0.1 0.6 -0.1 0.6 1.0 1.8]);
    view([45 45]);

    for i = 1:nKinect
        pl1 = plot3(mocapPos(i,1), mocapPos(i,2), mocapPos(i,3), '.b', 'MarkerSize', markerSize);
        pl2 = plot3(kinOut(i,1), kinOut(i,2), kinOut(i,3), '.r', 'MarkerSize', markerSize);

        drawnow;
        pause(0.1);
        delete(pl1);
        delete(pl2);
    end

    plot3(mocapPos(:,1), mocapPos(:,2), mocapPos(:,3), '.b', 'MarkerSize', markerSize);
    plot3(kinOut(:,1), kinOut(:,2), kinOut(:,3), '.r', 'MarkerSize', markerSize);
    hold off;

elseif mode == 1
    % Baxter Calibration Mode
    [~,baxterData] = parseBaxter(sprintf('%sEE',fileName));
    [~,mocapData] = parseOptitrack(sprintf('%s.csv',fileName));

    baxterData = baxterData(1:offset:end,:);

    mocapT = mocapData(:,2);
    baxterT = baxterData(:,1);

    nBaxter = size(baxterData,1);
    nMarkers = (size(mocapData,2) - 2)/3;

    mocapInd = zeros(nBaxter,1);
    for j = 1:nBaxter
        tRef = baxterT(j);
        [~,ind] = min((tRef - mocapT).^2);
        mocapInd(j) = ind;
    end
    baxterPos = baxterData(:,2:4);
    mocapPos = zeros(nBaxter,3);

    for j = 1:nMarkers
        mocapPos(:,1) = mocapPos(:,1) + mocapData(mocapInd,3+(j-1)*3);
        mocapPos(:,2) = mocapPos(:,2) + mocapData(mocapInd,4+(j-1)*3);
        mocapPos(:,3) = mocapPos(:,3) + mocapData(mocapInd,5+(j-1)*3);
    end
    mocapPos = mocapPos./nMarkers;

    [R,T,c,err,bOut] = absoluteOrientationSVD(baxterPos',mocapPos');
    baxterOut = bOut';
    transMatrix = [c*R; 0 0 0];
    transMatrix = [transMatrix [T; 1]];
    dlmwrite('BaxterCalibration', transMatrix);
    fprintf('Error: %f\n',err);

    figure;
    hold on;
    xlabel('X [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    ylabel('Y [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    zlabel('Z [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    title('Baxter Calibration',  'FontSize', fontSize, 'FontWeight', 'bold');
    set(gca, 'FontSize', fontSize, 'FontWeight', 'bold');
    axis([-1 1 0.5 2.5 -1 1]);
    view([45 45]);

    plot3(mocapPos(:,1), mocapPos(:,2), mocapPos(:,3), '.b', 'MarkerSize', markerSize);
    plot3(baxterOut(:,1), baxterOut(:,2), baxterOut(:,3), '.r', 'MarkerSize', markerSize);
    hold off;
end
