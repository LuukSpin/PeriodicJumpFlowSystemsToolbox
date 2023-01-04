function objSD_reconstructed = applyReconstructor(objSD, opts)
arguments
    objSD   (1,1) OpenLoopSampledDataSystem
    opts    (1,1) SDopts = SDopts(1)
end

if strcmpi(objSD.reconstructor, 'unspecified')
    % Dimensions
    nx = objSD.nx;
    nwc = objSD.nwc;
    nwd = objSD.nwd;
    nzc = objSD.nzc;
    nzd = objSD.nzd;
    nu = objSD.nu;
    ny = objSD.ny;

    if strcmpi(opts.reconstructor, 'zoh')
        % Determine new state-space matrices based on reconstructor
        Ac_h = zeros(nu);
        Ad_h = zeros(nu);
        Bd_h = eye(nu);
        Cc_h = eye(nu);
        nx_h = nu;

    elseif strcmpi(opts.reconstructor, 'foh')
        h = opts.simulation.SampleTime;
        Ac_h = [zeros(nu, 3*nu); [eye(nu), zeros(nu), -eye(nu)]/h; zeros(nu, 3*nu)];
        Ad_h = [zeros(nu, 3*nu); [[eye(nu); eye(nu)], zeros(2*nu, 2*nu)]];
        Bd_h = [eye(nu); zeros(2*nu, nu)];
        Cc_h = [zeros(nu), eye(nu), zeros(nu)];
        nx_h = 3*nu;
    else
        error('Only the reconstructor "ZOH" and "FOH" is defined at this moment in time.');
    end

    Ac = [objSD.Ac, objSD.Buc*Cc_h; zeros(nx_h, nx), Ac_h];
    Bwc = [objSD.Bwc; zeros(nx_h, nwc)];
    Buc = zeros(nx+nx_h, nu);

    % Jump matrices
    Ad = blkdiag(objSD.Ad, Ad_h);
    Bwd = [objSD.Bwd; zeros(nx_h, nwd)];
    Bud = [objSD.Bud; Bd_h];

    % Continuous-time performance channels matrices
    Czc = [objSD.Czc, objSD.Dzc_uc*Cc_h];
    Dzc_wc = objSD.Dzc_wc;
    Dzc_uc = zeros(nzc, nu);

    % Discrete-time performance channels matrices
    Czd = [objSD.Czd, zeros(nzd, nx_h)];
    Dzd_wd = objSD.Dzd_wd;
    Dzd_ud = objSD.Dzd_ud;

    % Controller input matrices
    Cy = [objSD.Cy, zeros(ny, nx_h)];
    Dy_wd = objSD.Dy_wd;
    Dy_ud = objSD.Dy_ud;

    objSD_reconstructed = OpenLoopSampledDataSystem(Ac, Bwc, Buc, Ad, Bwd, Bud, Czc, Dzc_wc, Dzc_uc, Czd, Dzd_wd, Dzd_ud, Cy, Dy_wd, Dy_ud);
    objSD_reconstructed.reconstructor = opts.reconstructor;

end
end