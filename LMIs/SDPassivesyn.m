function [Controller, PassiveFlag, CLJFSystem] = SDPassivesyn(OpenLoopSDSystem, opts)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
%

arguments
    OpenLoopSDSystem    (1,1) OpenLoopSampledDataSystem
    opts                (1,1) SDopts
end

dimCheck(OpenLoopSDSystem);

if strcmpi(OpenLoopSDSystem.reconstructor, 'unspecified')
    OpenLoopSDSystem = OpenLoopSDSystem.applyReconstructor(opts);
end

h = opts.simulation.SampleTime;
numAcc = opts.LMI.numericalAccuracy;

% Dimensions;
nx = OpenLoopSDSystem.nx;
nc = nx;
nu = OpenLoopSDSystem.nu;
ny = OpenLoopSDSystem.ny;

% LMI variables
Y = sdpvar(nx, nx, 'symmetric');
X = sdpvar(nc, nc, 'symmetric');
Gamma = sdpvar(nx, nc, 'full');
Theta = sdpvar(nx, ny, 'full');
Upsilon = sdpvar(nu, nc, 'full');
Omega = sdpvar(nu, ny, 'full');


% LMI variables
sdpVariables.Y = Y;
sdpVariables.X = X;
sdpVariables.Gamma = Gamma;
sdpVariables.Theta = Theta;
sdpVariables.Upsilon = Upsilon;
sdpVariables.Omega = Omega;

% Fill the H-infinity LMI
[HinfLMIMatrix, A_bar] = fillPassiveLMI(OpenLoopSDSystem, sdpVariables, opts);
HinfLMI = HinfLMIMatrix >= numAcc*eye(size(HinfLMIMatrix));

% Add a constraint
constraint = [sdpVariables.Y, eye(nx, nc); eye(nc, nx), sdpVariables.X] >= numAcc*eye(size(blkdiag(sdpVariables.Y, sdpVariables.X)));

if strcmpi(opts.LMI.schurController, 'yes')
    schurControllerLMI = [sdpVariables.Y, eye(size(sdpVariables.Y)); eye(size(sdpVariables.Y)), numAcc*10*eye(size(sdpVariables.Y))] >= -1e3*numAcc*eye(size(sdpVariables.Y)*2);
    LMI = [HinfLMI, constraint, schurControllerLMI];
else
    LMI = [HinfLMI, constraint];
end

diagnostics = optimize(LMI, [], opts.LMI.solverOptions);

% Determine if bisection-based search was succesful
if diagnostics.problem == 0
    PassiveFlag = true;
else
    PassiveFlag = false;
    Controller = c2d(ss(0), h);
    CLJFSystem = JumpFlowSystem();
    warning('No stabilizing controller is synthesized.');
    return
end

Controller = controllerConstruction(OpenLoopSDSystem, A_bar, sdpVariables, opts);

K_zpk = zpk(Controller);
K_zeros = K_zpk.z{:};
K_poles = K_zpk.p{:};

nrUnstabPole = length(K_poles(abs(K_poles)>1+eps));
nrNonMinPhaseZero = length(K_zeros(abs(K_zeros)>1+eps));

if nrUnstabPole+nrNonMinPhaseZero>0
    sprintf('The synthesized controller contains %d no. of unstable poles and %d no. of non-minimum phase zeros.', nrUnstabPole, nrNonMinPhaseZero);
end

CLJFSystem = OpenLoopSDSystem.lft(Controller, opts);

end