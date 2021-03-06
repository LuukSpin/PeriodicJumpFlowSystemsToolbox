function OLSDSystem = makeSDFromDiscreteController(discreteController, opts)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

arguments
    discreteController  {mustBeA(discreteController, ["ss", "tf"])}
    opts                (1,1) SDopts = SDopts()
end

% discreteController = minreal(ss(discreteController), [], false);

% Check if the controller is discrete-time controller
if discreteController.Ts == 0
    error('The controller has to be discrete-time controller');
end

% Controller matrices
Acontroller = discreteController.A;
Bcontroller = discreteController.B;
Ccontroller = discreteController.C;
Dcontroller = discreteController.D;

% Dimensions
nx = size(Acontroller, 1);
nu = size(Bcontroller, 2);
ny = size(Ccontroller, 1);
nwc = 0;
nwd = 0;
nzc = 0;
nzd = 0;

% Flow matrices
Ac = zeros(nx);
Bwc = zeros(nx, nwc);
Buc = zeros(nx, nu);

% Jump matrices
Ad = Acontroller;
Bwd = zeros(nx, nwd);
Bud = Bcontroller;

% Continuous-time performance channel matrices
Czc = zeros(nzc, nx);
Dzc_wc = zeros(nzc, nwc);
Dzc_uc = zeros(nzc, nu);

% Discrete-time performance channel matrices
Czd = zeros(nzd, nx);
Dzd_wd = zeros(nzd, nwd);
Dzd_u = zeros(nzd, nu);

% Controller input matrices
Cy = Ccontroller;
Dy_wd = zeros(ny, nwd);
Dy_u = Dcontroller;

OLSDSystem = OpenLoopSampledDataSystem(Ac, Bwc, Buc, Ad, Bwd, Bud, Czc, Dzc_wc, Dzc_uc, Czd, Dzd_wd, Dzd_u, Cy, Dy_wd, Dy_u);
end