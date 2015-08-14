% Matlab code to parse Kinect marker tracker output
% Author: Nishanth Koganti
% Date: 2015/08/14

function [Head, Data] = parseKinect(fileName)

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
