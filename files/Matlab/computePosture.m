% computePosture.m: Program to plot posture of mannequin
% Author: Nishanth Koganti
% Date: 2015/9/19

% TODO:
% 1) Implement in python

%% Function to plot body points
function [data, head, body, lShoulder, rShoulder] = computePosture(fileName, nPoints)

lineWidth = 3;
fontSize = 15;
markerSize = 20;
plotFlag = false;
printFlag = false;

[~,data] = parsePosture(fileName);

headPoints = [data(1,:); data(2,:)];
bodyPoints = [data(2,:); data(3,:)];
lShoulderPoints = [data(4,:); data(6,:)];
rShoulderPoints = [data(5,:); data(7,:)];

head = interpolate(headPoints, nPoints);
body = interpolate(bodyPoints, nPoints);
lShoulder = interpolate(lShoulderPoints, nPoints);
rShoulder = interpolate(rShoulderPoints, nPoints);

if plotFlag
    fid = figure;
    hold on;
    xlabel('X [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    ylabel('Y [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    zlabel('Z [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    title(sprintf('Body Posture %s',fileName),  'FontSize', fontSize, 'FontWeight', 'bold');
    set(gca, 'FontSize', fontSize, 'FontWeight', 'bold');
    axis([-0.2 1.0 -0.2 0.7 0.5 1.5]);
    view([150,30]);

    plot3(data(:,1), data(:,2), data(:,3), '.r', 'MarkerSize', markerSize);
    plot3(data(1:3,1), data(1:3,2), data(1:3,3), '-b', 'LineWidth', lineWidth);
    plot3(data([4,2,5],1), data([4,2,5],2), data([4,2,5],3), '-b', 'LineWidth', lineWidth);
    plot3(data([4,6],1), data([4,6],2), data([4,6],3), '-b', 'LineWidth', lineWidth);
    plot3(data([5,7],1), data([5,7],2), data([5,7],3), '-b', 'LineWidth', lineWidth);
    hold off;

    if printFlag
        print(fid,sprintf('../Results/Posture/%s.eps',fileName),'-depsc');
    end

    pause(2);
    close all;
end

%% Function to interpolate linearly between two given points
function output = interpolate(points,num)

tt = 1:2;
ti = 1:1/(num-1):2;

xInt = interp1(tt,points(:,1),ti);
yInt = interp1(tt,points(:,2),ti);
zInt = interp1(tt,points(:,3),ti);

output = transpose([xInt;yInt;zInt]);