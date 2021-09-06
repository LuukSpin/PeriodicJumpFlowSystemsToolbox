function objJF = convertSD2JF(objSD)
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here

% Make sure that the input is a sampled-data object
if ~(isa(objSD, 'OpenLoopSampledDataSystem') || isa(objSD, 'SampledDataSystem'))
    error('The input should be a sampled-data object');
end

% Check whether the sampled-data object is in open-loop
if strcmpi(objSD.Loop, 'Open')
    % Initiate JumpFlowSystem
    objJF = OpenLoopJumpFlowSystem();
    
    % Dimensions
    nu = size(objSD.Buc, 2);
    nx = size(objSD.Ac, 1);
    nwc = size(objSD.Bwc, 2);
    nwd = size(objSD.Bwd, 2);
    nzd = size(objSD.Czd, 1);
    ny = size(objSD.Cy, 1);
    
    % Flow matrices
    objJF.Ac = [objSD.Ac, objSD.Buc; zeros(nu, nx), zeros(nu)];
    objJF.Bwc = [objSD.Bwc; zeros(nu, nwc)];
    
    % Jump matrices
    objJF.Ad = blkdiag(objSD.Ad, zeros(nu));
    objJF.Bwd = [objSD.Bwd; zeros(nu, nwd)];
    objJF.Bud = [objSD.Bud; eye(nu)];
    
    % Continuous-time performance channels matrices
    objJF.Czc = [objSD.Czc, objSD.Dzc_u];
    objJF.Dzc_wc = objSD.Dzc_wc;
    
    % Discrete-time performance channels matrices
    objJF.Czd = [objSD.Czd, zeros(nzd, nu)];
    objJF.Dzd_wd = objSD.Dzd_wd;
    objJF.Dzd_u = objSD.Dzd_u;
    
    % Controller input matrices
    objJF.Cy = [objSD.Cy, zeros(ny, nu)];
    objJF.Dy_wd = objSD.Dy_wd;
    objJF.Dy_u = objSD.Dy_u;
    
elseif strcmpi(obj.SD.Loop, 'Closed')
    % Initiate JumpFlowSystem
    objJF = JumpFlowSystem();
    
    % Flow matrices
    objJF.Ac = objSD.Ac;
    objJF.Bwc = objSD.Bwc;
    
    % Jump matrices
    objJF.Ad = objSD.Ad;
    objJF.Bwd = objSD.Bwd;
    
    % Continuous-time performance channels matrices
    objJF.Czc = objSD.Czc;
    objJF.Dzc_wc = objSD.Dzc_wc;
    
    % Discrete-time performance channels matrices
    objJF.Czd = objSD.Czd;
    objJF.Dzd_wd = objSD.Dzd_wd;
    
else
    error('The Loop property of the sampled-data object is not "Closed" or "Open"');
end
end

