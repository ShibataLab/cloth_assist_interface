% plotPosture.m: Program to plot posture of mannequin
% Author: Nishanth Koganti
% Date: 2015/9/19

% TODO:
% 1) Implement in python

function plotPosture(fileName)

lineWidth = 3;
fontSize = 15;
markerSize = 20;
printFlag = true;

[~,data] = parsePosture(fileName);

fid = figure;
hold on;
xlabel('X [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
ylabel('Y [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
zlabel('Z [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
title('Body Posture',  'FontSize', fontSize, 'FontWeight', 'bold');
set(gca, 'FontSize', fontSize, 'FontWeight', 'bold');
axis([0.0 1.0 0.0 0.7 0.5 1.5]);
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

pause;
close all;