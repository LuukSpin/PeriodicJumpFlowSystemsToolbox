function [Controller, gamma, CLJFSystem] = SDHinfsyn(OpenLoopSDSystem, h, opts)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
%

arguments
    OpenLoopSDSystem    (1,1) OpenLoopSampledDataSystem
    h                   (1,1) double
    opts                (1,1) SDopts = SDopts();
end

dimCheck(OpenLoopSDSystem);

if strcmpi(OpenLoopSDSystem.reconstructor, 'unspecified')
    OpenLoopSDSystem = OpenLoopSDSystem.applyReconstructor(opts);
end

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

% Bisection settings
Nmax = 100; %Maximum numbers of iterations
tol = 1e-4; %Tolerance to calculate upperbound of Hinf norm

% Initialization
a_init = max(norm(OpenLoopSDSystem.Dzc_wc, 2));
if a_init == 0
    a = [tol 2];
else
    a = [a_init 2];
end

N = 0;
last = 0;
initialfeas = 0;

% Run the bisection based search until gamma is within the specified
% tolerance or the maximum amount of iterations is reached
while N < Nmax
    N = N + 1;
    
    if ~xor((a(2) - a(1)) > tol, N < Nmax-1) || last
        gamma = mean(a);
        
        % LMI variables
        sdpVariables.Y = Y;
        sdpVariables.X = X;
        sdpVariables.Gamma = Gamma;
        sdpVariables.Theta = Theta;
        sdpVariables.Upsilon = Upsilon;
        sdpVariables.Omega = Omega;

        % Fill the H-infinity LMI
        [HinfLMIMatrix, A_bar] = fillHinfLMI(OpenLoopSDSystem, sdpVariables, h, gamma);
        HinfLMI = HinfLMIMatrix >= numAcc*eye(size(HinfLMIMatrix));
        
        % Add a constraint
        constraint = [sdpVariables.Y, eye(nx, nc); eye(nc, nx), sdpVariables.X] >= numAcc*eye(size(blkdiag(sdpVariables.Y, sdpVariables.X)));

        if strcmpi(opts.LMI.schurController, 'yes')
            schurControllerLMI = [sdpVariables.Y, eye(size(sdpVariables.Y)); eye(size(sdpVariables.Y)), numAcc*10*eye(size(sdpVariables.Y))] >= -1e3*numAcc*eye(size(sdpVariables.Y)*2);
            LMI = [HinfLMI, constraint, schurControllerLMI];
        else
            LMI = [HinfLMI, constraint];
        end
        
        rng(1);
        diagnostics = optimize(LMI, [], opts.LMI.solverOptions);
        
        if diagnostics.problem == 0
            a(2) = mean(a);
            initialfeas = 1;
        else
            if initialfeas
                a(1) = mean(a);
            else
                a(2) = 2*a(2);
            end
        end
        if last
            break;
        end
    else
        last = 1;
        a(1) = a(2);
    end
    
end

% Determine if bisection-based search was succesful
if initialfeas
    gamma = a(2);
else
    gamma = nan;
    Controller = c2d(ss(0), h);
    CLJFSystem = JumpFlowSystem();
    warning('No stabilizing controller is synthesized.');
    return
end

if strcmpi(opts.LMI.controllerConditioning, 'yes')
    [sdpVariables, A_bar] = controllerConditioning(OpenLoopSDSystem, gamma, sdpVariables, h, 'Hinf', opts);
end

Controller = controllerConstruction(OpenLoopSDSystem, A_bar, sdpVariables, h);

K_zpk = zpk(Controller);
K_zeros = K_zpk.z{:};
K_poles = K_zpk.p{:};

nrUnstabPole = length(K_poles(abs(K_poles)>1+eps));
nrNonMinPhaseZero = length(K_zeros(abs(K_zeros)>1+eps));

if nrUnstabPole+nrNonMinPhaseZero>0
    sprintf('The synthesized controller contains %d no. of unstable poles and %d no. of non-minimum phase zeros.', nrUnstabPole, nrNonMinPhaseZero);
end

CLJFSystem = OpenLoopSDSystem.lft(Controller);

end