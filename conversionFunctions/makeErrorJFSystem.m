function ErrorJFSystem = makeErrorJFSystem(ClosedLoopJFReferenceSystem, OpenLoopJFModel)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

n_zc = size(OpenLoopJFModel.Czc, 1);
n_zc_ref = size(ClosedLoopJFReferenceSystem.Czc, 1);

if n_zc ~= n_zc_ref
    error('The dimensions of the continuous-time performance channels do no match between the open-loop JF model and reference closed-loop JF system.');
end

n_zd = size(OpenLoopJFModel.Czd, 1);
n_zd_ref = size(ClosedLoopJFReferenceSystem.Czd, 1);

if n_zd ~= n_zd_ref
    error('The dimensions of the discrete-time performance channels do no match between the open-loop JF model and reference closed-loop JF system.');
end

n_wc = size(OpenLoopJFModel.Bwc, 2);
n_wc_ref = size(ClosedLoopJFReferenceSystem.Bwc, 2);

if n_wc ~= n_wc_ref
    error('The dimensions of the continuous-time disturbance channels do no match between the open-loop JF model and reference closed-loop JF system.');
end

n_wd = size(OpenLoopJFModel.Bwd, 2);
n_wd_ref = size(ClosedLoopJFReferenceSystem.Bwd, 2);

if n_wd ~= n_wd_ref
    error('The dimensions of the discrete-time disturbance channels do no match between the open-loop JF model and reference closed-loop JF system.');
end

ny = size(OpenLoopJFModel.Cy, 1);
nu = size(OpenLoopJFModel.Bud, 2);
nref = size(ClosedLoopJFReferenceSystem.Ac, 1);

%Calculate closed-loop flow matrices from the open-loop JF system

%Initiate new JF class for the error system
ErrorJFSystem = OpenLoopJumpFlowSystem();

%Flow matrices
ErrorJFSystem.Ac = blkdiag(ClosedLoopJFReferenceSystem.Ac, OpenLoopJFModel.Ac);
ErrorJFSystem.Bwc = [ClosedLoopJFReferenceSystem.Bwc; OpenLoopJFModel.Bwc];

%Jump matrices
ErrorJFSystem.Ad = blkdiag(ClosedLoopJFReferenceSystem.Ad, OpenLoopJFModel.Ad);
ErrorJFSystem.Bwd = [ClosedLoopJFReferenceSystem.Bwd; OpenLoopJFModel.Bwd];
ErrorJFSystem.Bud = [zeros(nref, nu); OpenLoopJFModel.Bud];

%Continuous-time performance channel matrices
ErrorJFSystem.Czc = [-ClosedLoopJFReferenceSystem.Czc, OpenLoopJFModel.Czc];
ErrorJFSystem.Dzc_wc = -ClosedLoopJFReferenceSystem.Dzc_wc+OpenLoopJFModel.Dzc_wc;

%Discrete-time performance channel matrices
ErrorJFSystem.Czd = [-ClosedLoopJFReferenceSystem.Czd, OpenLoopJFModel.Czd];
% checkZeroDzd_wdRef = all(ClosedLoopJFReferenceSystem.Dzd_wd==0, 'all');
% if ~checkZeroDzd_wdRef
%     error('The matrix Dzd_wd of the ref model should be 0!');
% end
ErrorJFSystem.Dzd_wd = OpenLoopJFModel.Dzd_wd; %The matrix Dzd_wd of the ref model should be 0!
ErrorJFSystem.Dzd_u = OpenLoopJFModel.Dzd_u;

%Disrete-time output matrices
ErrorJFSystem.Cy = [zeros(ny, nref), OpenLoopJFModel.Cy];
ErrorJFSystem.Dy_wd = OpenLoopJFModel.Dy_wd;
ErrorJFSystem.Dy_u = OpenLoopJFModel.Dy_u;
end