function SDSystem = unweightedOLSDSystem(Ac, Bwc, Buc, Czc, Dzc_wc, Dzc_u, Czd, Dzd_wd, Dzd_u, Cy, Dy_wd)

% Initiate sampled-data system
SDSystem = OpenLoopSampledDataSystem();

% Dimensions
nx = size(Ac, 1);
nu = size(Buc, 2);
ny = size(Cy, 1);
n_wd = size(Dzd_wd, 2);

%Flow matrices
SDSystem.Ac = Ac;
SDSystem.Bwc = Bwc;
SDSystem.Buc = Buc;

%Jump matrices
SDSystem.Ad = eye(nx);
SDSystem.Bwd = zeros(nx, n_wd);
SDSystem.Bud = zeros(nx, nu);

%Continuous-time performance channels
SDSystem.Czc = Czc;
SDSystem.Dzc_wc = Dzc_wc;
SDSystem.Dzc_u = Dzc_u;

%Discrete-time performance channels
SDSystem.Czd = Czd;
SDSystem.Dzd_wd = Dzd_wd;
SDSystem.Dzd_u = Dzd_u;

%Discrete-time output matrices
SDSystem.Cy = Cy; %This is the sampled output of G
SDSystem.Dy_wd = Dy_wd;
SDSystem.Dy_u = zeros(ny, nu);

end

