function weightedSD = appendWeightingFilters(objSD, Vc, Vd, Wc, Wd)
arguments
    objSD      OpenLoopSampledDataSystem
    Vc         {mustBeNumericOrListedType(Vc, "ss", "tf")} = 1
    Vd         {mustBeNumericOrListedType(Vd, "ss", "tf")} = 1
    Wc         {mustBeNumericOrListedType(Wc, "ss", "tf")} = 1
    Wd         {mustBeNumericOrListedType(Wd, "ss", "tf")} = 1
end

% Check if filters are continuous- or discrete-time filters
if isnumeric(Vc)
    Vc = ss(Vc);
else
    if Vc.Ts ~= 0
        error('The weighting filter "Vc" for the continuous-time disturbance channels should be a continuous-time filter.');
    end
    Vc = minreal(Vc, [], false);
end

if isnumeric(Vd)
    Vd = c2d(ss(Vd), 1);
else
    if Vd.Ts == 0
        error('The weighting filter "Vd" for the discrete-time disturbance channels should be a discrete-time filter.');
    end
    Vd = minreal(Vd, [], false);
end

if isnumeric(Wc)
    Wc = ss(Wc);
else
    if Wc.Ts ~= 0
        error('The weighting filter "Wc" for the continuous-time performance channels should be a continuous-time filter.');
    end
    Wc = minreal(Wc, [], false);
end

if isnumeric(Wd)
    Wd = c2d(ss(Wd), 1);
else
    if Wd.Ts == 0
        error('The weighting filter "Wd" for the discrete-time disturbance channels should be a discrete-time filter.');
    end
    Wd = minreal(ss(Wd), [], false);
end

% Check dimensions
dimCheck(objSD);
nwc = objSD.nwc;
nwd = objSD.nwd;
nzc = objSD.nzc;
nzd = objSD.nzd;
nu  = objSD.nu;
ny  = objSD.ny;

if nwc ~= size(Vc, 1)
    error('The number of outputs of the filter "Vc" should be equal to nwc');
elseif nwc ~= size(Vc, 2)
    error('The number of inputs of the filter "Vc" should be equal to nwc');
end

if nwd ~= size(Vd, 1)
    error('The number of outputs of the filter "Vd" should be equal to nwd');
elseif nwd ~= size(Vd, 2)
    error('The number of inputs of the filter "Vd" should be equal to nwd');
end

if nzc ~= size(Wc, 1)
    error('The number of outputs of the filter "Wc" should be equal to nzc');
elseif nzc ~= size(Wc, 2)
    error('The number of inputs of the filter "Wc" should be equal to nzc');
end

if nzd ~= size(Wd, 1)
    error('The number of outputs of the filter "Wd" should be equal to nzd');
elseif nzd ~= size(Wd, 2)
    error('The number of inputs of the filter "Wd" should be equal to nzd');
end

% State dimensions
nx_g = objSD.nx;
nx_wc = size(Vc.A, 1);
nx_zc = size(Wc.A, 1);
nx_wd = size(Vd.A, 1);
nx_zd = size(Wd.A, 1);

% Define state-space matrices of the weighed system
% Flow matrices
Ac  = [objSD.Ac, objSD.Bwc*Vc.C, zeros(nx_g, nx_wd+nx_zc+nx_zd);...
    zeros(nx_wc, nx_g), Vc.A, zeros(nx_wc, nx_wd+nx_zc+nx_zd);...
    zeros(nx_wd, nx_g+nx_wc+nx_zc+nx_wd+nx_zd);...
    Wc.B*objSD.Czc, Wc.B*objSD.Dzc_wc*Vc.C, zeros(nx_zc, nx_wd), Wc.A, zeros(nx_zc, nx_zd);...
    zeros(nx_zd, nx_g+nx_wc+nx_zc+nx_wd+nx_zd)];
Bwc = [objSD.Bwc*Vc.D; Vc.B; zeros(nx_wd, nwc); Wc.B*objSD.Dzc_wc*Vc.D; zeros(nx_zd, nwc)];
Buc = [objSD.Buc; zeros(nx_wc + nx_wd, nu); Wc.B*objSD.Dzc_uc; zeros(nx_zd, nu)];

% Jump matrices
Ad  = [objSD.Ad, zeros(nx_g, nx_wc), objSD.Bwd*Vd.C, zeros(nx_g, nx_zc+nx_zd);...
    zeros(nx_wc, nx_g), eye(nx_wc), zeros(nx_wc, nx_wd+nx_zc+nx_zd);...
    zeros(nx_wd, nx_g+nx_wc), Vd.A, zeros(nx_wd, nx_zc+nx_zd);...
    zeros(nx_zc, nx_g+nx_wc+nx_wd), eye(nx_zc), zeros(nx_zc, nx_zd);...
    Wd.B*objSD.Czd, zeros(nx_zd, nx_wc), Wd.B*objSD.Dzd_wd*Vd.C, zeros(nx_zd, nx_zc), Wd.A];
Bwd = [objSD.Bwd*Vd.D; zeros(nx_wc, nwd); Vd.B; zeros(nx_zc, nwd); Wd.B*objSD.Dzd_wd*Vd.D];
Bud = [objSD.Bud; zeros(nx_wc+nx_wd+nx_zc, nu); Wd.B*objSD.Dzd_ud];

% Continuous-time performance channel matrices
Czc = [Wc.D*objSD.Czc, Wc.D*objSD.Dzc_wc*Vc.C, zeros(nzc, nx_wd), Wc.C, zeros(nzc, nx_zd)];
Dzc_wc = Wc.D*objSD.Dzc_wc*Vc.D;
Dzc_uc = Wc.D*objSD.Dzc_uc;

% Discrete-time performance channel matrices
Czd = [Wd.D*objSD.Czd, zeros(nzd, nx_wc), Wd.D*objSD.Dzd_wd*Vd.C, zeros(nzd, nx_zc), Wd.C];
Dzd_wd = Wd.D*objSD.Dzd_wd*Vd.D;
Dzd_ud = Wd.D*objSD.Dzd_ud;

% Controller input matrices
Cy = [objSD.Cy, zeros(ny, nx_wc), objSD.Dy_wd*Vd.C, zeros(ny, nx_zc+nx_zd)];
Dy_wd = objSD.Dy_wd*Vd.D;
Dy_ud = objSD.Dy_ud;

weightedSD = OpenLoopSampledDataSystem(Ac, Bwc, Buc, Ad, Bwd, Bud, Czc, Dzc_wc, Dzc_uc, Czd, Dzd_wd, Dzd_ud, Cy, Dy_wd, Dy_ud);
weightedSD.reconstructor = objSD.reconstructor;
end