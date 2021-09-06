function [A, B, C, D] = closedLoopFlowMatrices(JFSystem, nc)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

ni = nargin;

checkOL = strcmpi(JFSystem.Loop, 'Open');
if ~checkOL
    error('Closed-loop flow matrices can only be determine for a open-loop system');
end

%dimensions
nx = size(JFSystem.Ac, 1); %This is equal to nx of the sampled-data system plus nu
n_wc = size(JFSystem.Bwc, 2);
n_zc = size(JFSystem.Czc, 1);

if ni<2
    nc = nx;
end

A = blkdiag(JFSystem.Ac, zeros(nc));
B = [JFSystem.Bwc; zeros(nc, n_wc)];
C = [JFSystem.Czc, zeros(n_zc, nc)];
D = JFSystem.Dzc_wc;

end

