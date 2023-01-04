function objJF = convertSD2JF(objSD, optsSD)
%CONVERTSD2JF converts sampled-data object to a jump-flow object based on
%the reconstructor specified.
%   Detailed explanation goes here

arguments
    objSD (1,1) {mustBeA(objSD, ["SampledDataSystem", "OpenLoopSampledDataSystem"])}
    optsSD (1,1) SDopts = SDopts(1);
end

% Check whether the sampled-data object is in open-loop
if strcmpi(objSD.Loop, 'Open')
    % Initiate JumpFlowSystem

    % Dimensions
    nu = objSD.nu;
    nx = objSD.nx;
    nwc = objSD.nwc;
    nwd = objSD.nwd;
    nzc = objSD.nzc;
    nzd = objSD.nzd;
    ny = objSD.ny;

    if strcmpi(optsSD.reconstructor, 'ZOH')

        % Flow matrices
        Ac = [objSD.Ac, objSD.Buc; zeros(nu, nx), zeros(nu)];
        Bwc = [objSD.Bwc; zeros(nu, nwc)];
        Buc = zeros(size(Ac, 1), nu);

        % Jump matrices
        Ad = blkdiag(objSD.Ad, zeros(nu));
        Bwd = [objSD.Bwd; zeros(nu, nwd)];
        Bud = [objSD.Bud; eye(nu)];

        % Continuous-time performance channels matrices
        Czc = [objSD.Czc, objSD.Dzc_uc];
        Dzc_wc = objSD.Dzc_wc;
        Dzc_uc = zeros(nzc, nu);

        % Discrete-time performance channels matrices
        Czd = [objSD.Czd, zeros(nzd, nu)];
        Dzd_wd = objSD.Dzd_wd;
        Dzd_ud = objSD.Dzd_ud;

        % Controller input matrices
        Cy = [objSD.Cy, zeros(ny, nu)];
        Dy_wd = objSD.Dy_wd;
        Dy_ud = objSD.Dy_ud;

        objJF = OpenLoopSampledDataSystem(Ac, Bwc, Buc, Ad, Bwd, Bud, Czc, Dzc_wc, Dzc_uc, Czd, Dzd_wd, Dzd_ud, Cy, Dy_wd, Dy_ud);

    else
        % At this point on the reconstructor ZOH is defined for this
        % toolbox.
        error('Only the reconstructor ZOH is defined at this point in time for this toolbox. In the future a FOH or n-th order hold could be implemented');
    end

elseif strcmpi(obj.SD.Loop, 'Closed')
    % Flow matrices
    Ac = objSD.Ac;
    Bwc = objSD.Bwc;

    % Jump matrices
    Ad = objSD.Ad;
    Bwd = objSD.Bwd;

    % Continuous-time performance channels matrices
    Czc = objSD.Czc;
    Dzc_wc = objSD.Dzc_wc;

    % Discrete-time performance channels matrices
    Czd = objSD.Czd;
    Dzd_wd = objSD.Dzd_wd;

    % Initiate JumpFlowSystem
    objJF = JumpFlowSystem(Ac, Bwc, Ad, Bwd, Czc, Dzc_wc, Czd, Dzd_wd);

else
    error('The Loop property of the sampled-data object is not "Closed" or "Open"');
end
end

