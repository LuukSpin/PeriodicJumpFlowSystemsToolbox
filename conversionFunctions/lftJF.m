function CLJFSystem = lftJF(OLJFSystem,discreteController)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

% Check if first input is an open-loop jump/flow system
if ~isa(OLJFSystem, 'OpenLoopSampledDataSystem')
    error('The input should be an open-loop jump/flow system');
end

% Check if second input is a ss object (or tf)
if ~isa(discreteController, 'ss')
    if isa(discreteController, 'tf')
        discreteController = ss(discreteController);
    else
        error('The controller has to be a "ss" or "tf" class object');
    end
    
    % Check if the controller is discrete-time controller
    if discreteController.Ts == 0
        error('The controller has to be discrete-time controller');
    end
end

% Controller matrices
Acontroller = discreteController.A;
Bcontroller = discreteController.B;
Ccontroller = discreteController.C;
Dcontroller = discreteController.D;
controllerMat = [Acontroller, Bcontroller; Ccontroller, Dcontroller];

% Dimensions
nx = OLJFSystem.nx;
nc = size(Acontroller, 1);
n_wd = OLJFSystem.nwd;
n_zd = OLJFSystem.nzd;
nu = OLJFSystem.nu;
ny = OLJFSystem.ny;

% Determine all closed-loop flow matrices
[Ac, Bwc, Czc, Dzc_wc] = OLJFSystem.ClosedLoopFlowMatrices(nc);

% Initiate closed-loop jump-flow system
CLJFSystem = JumpFlowSystem();

% Flow matrices
CLJFSystem.Ac = Ac;
CLJFSystem.Bwc = Bwc;

% Jump matrices
AdAdd = blkdiag(OLJFSystem.Ad, zeros(nc));
AdLeft = [zeros(nx, nc), OLJFSystem.Bud; eye(nc), zeros(nc, nu)];
AdRight = [zeros(nc, nx), eye(nc); OLJFSystem.Cy, zeros(ny, nc)];
CLJFSystem.Ad = AdAdd+AdLeft*controllerMat*AdRight;
BwdAdd = [OLJFSystem.Bwd; zeros(nc, n_wd)];
BwdLeft = AdLeft;
BwdRight = [zeros(nc, n_wd); OLJFSystem.Dy_wd];
CLJFSystem.Bwd = BwdAdd+BwdLeft*controllerMat*BwdRight;

% Continuous-time performance channel matrices
CLJFSystem.Czc = Czc;
CLJFSystem.Dzc_wc = Dzc_wc;

% Discrete-time performance channel matrices
CzdAdd = [OLJFSystem.Czd, zeros(n_zd, nc)];
CzdLeft = [zeros(n_zd, nc), OLJFSystem.Dzd_ud];
CzdRight = AdRight;
CLJFSystem.Czd = CzdAdd+CzdLeft*controllerMat*CzdRight;
Dzd_wdAdd = OLJFSystem.Dzd_wd;
Dzd_wdLeft = CzdLeft;
Dzd_wdRight = BwdRight;
CLJFSystem.Dzd_wd = Dzd_wdAdd+Dzd_wdLeft*controllerMat*Dzd_wdRight;

end

