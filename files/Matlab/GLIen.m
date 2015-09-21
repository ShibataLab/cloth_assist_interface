% GLIen.m: Program to compute Topology Coordinates between two strings
% Author: Nishanth Koganti
% Date: 2015/9/20

function params = GLIen(a,b)

%close all;

%Parameter Initialization
w = 0;
xd = 0;
yd = 0;

ag = size(a,1);
bg = size(b,1);
T = zeros(ag,bg);

%Computing Writhe Matrix
for i = 1:(ag-1)
    for j = 1:(bg-1)
                T(i,j) = GLI(a(i,:),a(i+1,:),b(j,:),b(j+1,:));
                w = w + (T(i,j));
                yd = yd + j*T(i,j);
                xd = xd + i*T(i,j);
    end
end

%Computing Center Topology Coordinate
xg = xd/w - bg/2;
yg = yd/w - ag/2;
c = [xg yg];

maxTi = zeros(1,2);
maxpointd1 = [0 0];
maxpointd2 = [0 0];

for i=1:(ag-1) 
    if T(i,1) >= maxTi(1)
        maxpointd1 = [i 1];
        maxTi(1) = T(i,1);
    end
    
    if T(i,bg-1) >= maxTi(2)
        maxpointd2 = [i bg-1];
        maxTi(2) = T(i,bg-1);
    end
end

diagonal = [-1 1];
maxpoint = [maxpointd1;maxpointd2];
principal = [(maxpointd2(1)-maxpointd1(1))/2 (maxpointd2(2)-maxpointd1(2))/2];
d = acos(dot(diagonal,principal)/(norm(diagonal)*norm(principal)));

%T=T';

params = [w c(1) c(2) d];

%figure;
%imagesc(T)
%colormap(flipud(gray));
%hold on;
%plot(c(1),c(2),'r.');
%plot(maxpoint(:,1),maxpoint(:,2),'b-.');
%hold off
