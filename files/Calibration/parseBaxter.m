% parseBaxter.m: Matlab code to parse baxter recorder output file
% Author: Nishanth Koganti
% Date: 2015/08/23

% TODO:
% 1) Implement in python

function [Head, Data] = parseBaxter(fileName)

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
