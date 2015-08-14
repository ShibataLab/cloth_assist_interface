% Matlab code to parse evart mocap trc files
% Author: Nishanth Koganti
% Date: 2015/08/14

function [Head, Data] = parseMocap(fileName)

fid = fopen(fileName);

% Getting the Header
Head = strsplit(fgets(fid),',');

% Getting the Data
Data = [];
while ~feof(fid)
      line = fgets(fid);
      dat = str2num(line);
      Data = [Data; dat];
end
fclose(fid);
