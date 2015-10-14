% parseClusters.m: Program to extract cluster centers from files and plot results
% Author: Nishanth Koganti
% Date: 2015/10/13

function [minVar, meanVar, maxVar] = analyzeClusters(fileName)

close all;

mode = 2;
fid = fopen(fileName);

%% plotting initialization
if mode == 2
    fontSize = 15;

    figure;
    hold on;
    title('Clusters Plotter', 'FontSize', fontSize, 'FontWeight', 'bold');
    xlabel('X', 'FontSize', fontSize, 'FontWeight', 'bold');
    ylabel('Y', 'FontSize', fontSize, 'FontWeight', 'bold');
    zlabel('Z', 'FontSize', fontSize, 'FontWeight', 'bold');
    set(gca, 'FontSize', fontSize, 'FontWeight', 'bold');
    view([45,45]);
end

%% parsing data
prevClusters = [];
clusterData = [];

waitforbuttonpress;

while ~feof(fid)
    line = fgets(fid);
    dat = str2num(line);
    nPoints = dat(2);
    clusters = zeros(nPoints,3);
    
    for i = 1:nPoints
        line = fgets(fid);
        dat = str2num(line);
        clusters(i,:) = dat;
    end
    
    % strategy 1
    % sort clusters column wise
    % clusters = sortrows(clusters);
    
    % strategy 2
    % sort clusters based on distance from origin or fixed point
    % clusterNorms = sqrt(sum(abs(clusters).^2,2));
    % [~, ind] = sort(clusterNorms);
    % clusters = clusters(ind,:);
    
    % strategy 3
    % sort clusters greedily based on previous cluster centers
    if ~isempty(prevClusters)
        tempMat = zeros(size(clusters));
    
        for i = 1:nPoints,
            point = prevClusters(i,:);
            dist = sqrt(sum((repmat(point,nPoints-i+1,1) - clusters).^2,2));
            [~,ind] = min(dist);
            tempMat(i,:) = clusters(ind,:);
            clusters(ind,:) = [];
        end
        
        clusters = tempMat;
    end
    prevClusters = clusters;
    
    clusterData = [clusterData; reshape(clusters', 1, size(clusters,1)*size(clusters,2))];
    
    if mode == 2
        pl1 = plot3(clusters(:,1), clusters(:,2), clusters(:,3), '.k', 'MarkerSize', 10);    
        pl2 = plot3(clusters(1,1), clusters(1,2), clusters(1,3), '.b', 'MarkerSize', 50);    
        pl3 = plot3(clusters(end,1), clusters(end,2), clusters(end,3), '.r', 'MarkerSize', 50);
        drawnow;
        pause(0.03);
        delete(pl1);
        delete(pl2);
        delete(pl3);
    end    
end

velData = [clusterData(2:end,:); clusterData(end,:)];
velData = (velData - clusterData)./0.033;

varTracks = var(velData);
maxVar = max(varTracks);
minVar = min(varTracks);
meanVar = mean(varTracks);

waitforbuttonpress;

if mode >= 1
    nDims = size(clusterData,2);
    nSamples = size(clusterData,1);

    xData = 1:nSamples;
    nFigs = ceil(nDims/9);
    
    for n = 1:nFigs
        figure;
        hold on;
        for i = 1:3,
            for j = 1:3,
                if ((n-1)*9 + (i-1)*3 + j) < nDims,
                    subplot(3,3,(i-1)*3+j);
                    plot(xData,clusterData(:,(n-1)*9 + (i-1)*3 + j),'-b','LineWidth',2);
                end
            end
        end
    end
end

fclose(fid);

return;