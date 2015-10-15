% analyzeClustersMain.m: Main program to process all the results
% Author: Nishanth Koganti
% Date: 2015/10/15

clusterNum = [10 20 30 40 50];
nTrials = length(clusterNum);

strat1 = zeros(nTrials, 3);
strat2 = zeros(nTrials, 3);
strat3 = zeros(nTrials, 3);
for i = 1:nTrials
    fileName = sprintf('clusters%d100',clusterNum(i));
    [strat1(i,:), ~] = analyzeClusters(fileName,1);
    [strat2(i,:), ~] = analyzeClusters(fileName,2);
    [strat3(i,:), tracks] = analyzeClusters(fileName,3);
    
    if i == 1
        varTracks1 = tracks;
    else
        varTracks2 = tracks;
    end
end

%% Plotting

fontSize = 15;
lineWidth = 2;
markerSize = 10;

figure;
hold on;
title('K-Means Cluster Alignment Strategies', 'FontSize', fontSize, 'FontWeight', 'bold');
xlabel('Number of Clusters', 'FontSize', fontSize, 'FontWeight', 'bold');
ylabel('Variance in velocity', 'FontSize', fontSize, 'FontWeight', 'bold');
set(gca, 'FontSize', fontSize, 'FontWeight', 'bold');

plot(clusterNum, strat1(:,3)', '--k', 'LineWidth', lineWidth);
h(1) = plot(clusterNum, strat1(:,2)', '-.k', 'LineWidth', lineWidth);
plot(clusterNum, strat2(:,3)', '--r', 'LineWidth', lineWidth);
h(2) = plot(clusterNum, strat2(:,2)', '-.r', 'LineWidth', lineWidth);
plot(clusterNum, strat3(:,3)', '--b', 'LineWidth', lineWidth);
h(3) = plot(clusterNum, strat3(:,2)', '-.b', 'LineWidth', lineWidth);
legend(h, 'Strategy 1', 'Strategy 2', 'Strategy 3', 'Location', 'NorthWest');
hold off;

figure;
hold on;
title('K-Means Cluster Variance: 10', 'FontSize', fontSize, 'FontWeight', 'bold');
xlabel('Cluster Number', 'FontSize', fontSize, 'FontWeight', 'bold');
ylabel('Variance in velocity', 'FontSize', fontSize, 'FontWeight', 'bold');
set(gca, 'FontSize', fontSize, 'FontWeight', 'bold');

plot(1:clusterNum(1), varTracks1(1:3:end), '-b', 'LineWidth', lineWidth);
hold off;

figure;
hold on;
title('K-Means Cluster Variance: 50', 'FontSize', fontSize, 'FontWeight', 'bold');
xlabel('Cluster Number', 'FontSize', fontSize, 'FontWeight', 'bold');
ylabel('Variance in velocity', 'FontSize', fontSize, 'FontWeight', 'bold');
set(gca, 'FontSize', fontSize, 'FontWeight', 'bold');

plot(1:clusterNum(end), varTracks2(1:3:end), '-b', 'LineWidth', lineWidth);
hold off;