% topologyCompute.m: Matlab code to compute Human-Cloth Topological Relationship
% Input Arguments:
% head, body, shoulder: nBodyx3 Matrix containing mannequin body coordinates
% collar, sleeve: nShapex3 Matrix containing collar and sleeve ellipse points
% Author: Nishanth Koganti
% Date: 2015/9/20

function params = topologyCompute(head,body,lShoulder,rShoulder,collar,lSleeve,rSleeve)   
%Topology Coordinate Estimation

%Collar-Head relationship
params1 = GLIen(collar,head);

%Collar-Body relatioship
params2 = GLIen(collar,body);

%Sleeve-Shoulder relationship
params3 = GLIen(lSleeve,lShoulder);

%Sleeve-Shoulder relationship
params4 = GLIen(rSleeve,rShoulder);

params = [params1([1 3]) params2([1 3]) params3([1 3]) params4([1 3])];