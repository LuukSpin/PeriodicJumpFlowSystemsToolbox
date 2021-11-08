function objSD = convertJF2SD(objJF, optsSD)
%CONVERTJF2SD converts a jump-flow object to a sampled-data object based on
%the reconstructor specified.
%   SD = CONVERTJF2SD(objJF, optsSD) converts an open-loop jump-flow system
%   to an open-loop sampled-data system.

arguments
    objJF (1,1) OpenLoopJumpFlowSystem
    optsSD (1,1) SDopts = SDopts();
end


% Dimensions
nuc = objJF.nuc;
nud = objJF.nud;
if nuc ~= nud
    error('In order for an open-loop jump-flow system to be converted to an open-loop sampled-data system the continuous- and discrete-time controller output dimensions must agree. That means that nuc must be equal to nud.');
end
nu = nuc;
nx = objJF.nx;
nwc = objJF.nwc;
nwd = objJF.nwd;
nzd = objJF.nzd;
nyd = objJF.nyd;

if strcmpi(optsSD.reconstructor, 'ZOH')

    % Flow matrices
    Ac = [objJF.Ac, objJF.Buc; zeros(nu, nx), zeros(nu)];
    Bwc = [objJF.Bwc; zeros(nu, nwc)];

    % Jump matrices
    Ad = blkdiag(objJF.Ad, zeros(nu));
    Bwd = [objJF.Bwd; zeros(nu, nwd)];
    Bud = [objJF.Bud; eye(nu)];

    % Continuous-time performance channels matrices
    Czc = [objJF.Czc, objJF.Dzc_uc];
    Dzc_wc = objJF.Dzc_wc;

    % Discrete-time performance channels matrices
    Czd = [objJF.Czd, zeros(nzd, nu)];
    Dzd_wd = objJF.Dzd_wd;
    Dzd_u = objJF.Dzd_ud;

    % Controller input matrices
    Cy = [objJF.Cyd, zeros(nyd, nu)];
    Dy_wd = objJF.Dyd_wd;
    Dy_u = objJF.Dyd_ud;

    objSD = OpenLoopSampledDataSystem(Ac, Bwc, Ad, Bwd, Bud, Czc, Dzc_wc, Czd, Dzd_wd, Dzd_u, Cy, Dy_wd, Dy_u);

else
    % At this point on the reconstructor ZOH is defined for this
    % toolbox.
    error('Only the reconstructor ZOH is defined at this point in time for this toolbox. In the future a FOH or n-th order hold could be implemented');
end

end

