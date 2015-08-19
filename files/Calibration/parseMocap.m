% Matlab code to parse evart mocap trc files
% Author: Nishanth Koganti
% Date: 2015/08/14

function [Head, Data] = parseMocap(fileName)

fid = fopen(fileName);

% Getting the Header
fgets(fid);
fgets(fid);
fgets(fid);
fgets(fid);
Head = strsplit(fgets(fid),'\t');
Head = circshift(Head,[0,1]);
Head{1} = 'Frame';
Head{2} = 'Time';

% Getting the Data
Data = [];
while ~feof(fid)
      line = fgets(fid);
      dat = str2num(line);
      Data = [Data; dat];
end

Data(:,3:end) = Data(:,3:end)/1000;
fclose(fid);
