function [Controller, gamma] = JFHinfsyn(OpenLoopJFSystem, h, opts)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
%

arguments
    OpenLoopJFSystem    (1,1) OpenLoopJumpFlowSystem
    h                   (1,1) double
    opts                (1,1) SDopts = SDopts();
end

backoff = opts.LMI.backoffFactor;
numAcc = opts.LMI.numericalAccuracy;

% Dimensions;
nx = OpenLoopJFSystem.nx;
nc = nx;
nu = OpenLoopJFSystem.nu;
ny = OpenLoopJFSystem.ny;

% LMI variables
sdpVariables.Y = sdpvar(nx, nx, 'symmetric');
sdpVariables.X = sdpvar(nc, nc, 'symmetric');
sdpVariables.Gamma = sdpvar(nx, nc, 'full');
sdpVariables.Theta = sdpvar(nx, ny, 'full');
sdpVariables.Upsilon = sdpvar(nu, nc, 'full');
sdpVariables.Omega = sdpvar(nu, ny, 'full');

% Bisection settings
Nmax = 100; %Maximum numbers of iterations
tol = 1e-4; %Tolerance to calculate upperbound of Hinf norm

% Initialization
a = [tol 2];
N = 0;
last = 0;
initialfeas = 0;

% Run the bisection based search until gamma is within the specified
% tolerance or the maximum amount of iterations is reached
while N < Nmax
    N = N + 1;
    
    if ~xor((a(2) - a(1)) > tol, N < Nmax-1) || last
        gamma = mean(a);
        
        % Fill the H-infinity LMI
        [HinfLMIMatrix, A_bar, Q_bar, Z_bar, W_bar] = fillHinfLMI(OpenLoopJFSystem, sdpVariables, h, gamma);
        HinfLMI = (HinfLMIMatrix+HinfLMIMatrix')/2 >= numAcc*eye(size(HinfLMIMatrix));
        
        % Add a constraint
        constraint = [sdpVariables.Y, eye(nx, nc); eye(nc, nx), sdpVariables.X] >= numAcc*eye(size(blkdiag(sdpVariables.Y, sdpVariables.X)));
        
        opts = LS.opts;
        rng(1);
        diagnostics = optimize(HinfLMI+constraint, [], opts.LMI);
        
        if (value(HinfLMI) && value(constraint))
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
    gamma = inf;
    Controller = ss(0);
    return
end

% Give values to the LMI variables in order to calculate the controller
Y_value = value(sdpVariables.Y);
X_value = value(sdpVariables.X);
Gamma_value = value(sdpVariables.Gamma);
Theta_value = value(sdpVariables.Theta);
Upsilon_value = value(sdpVariables.Upsilon);
Omega_value = value(sdpVariables.Omega);

U = X_value;
V = inv(X_value)-Y_value;

% Calculate controller
controllerMat = [V, Y_value*A_bar*Q_bar; zeros(nu, size(V, 2)), eye(nu)]\[Gamma_value-Y_value*A_bar*Z_bar*X_value, Theta_value; Upsilon_value, Omega_value]/[U', zeros(size(Y_value, 1), ny); W_bar*X_value, eye(ny)];
Controller = minreal(ss(controllerMat(1:nc, 1:nc), controllerMat(1:nc, nc+1:end), controllerMat(nc+1:end, 1:nc), controllerMat(nc+1:end, nc+1:end), h), [], false);

K_zpk = zpk(Controller);
K_zeros = K_zpk.z{:};
K_poles = K_zpk.p{:};

nrUnstabPole = length(K_poles(abs(K_poles)>1+eps));
nrNonMinPhaseZero = length(K_zeros(abs(K_zeros)>1+eps));

if nrUnstabPole+nrNonMinPhaseZero>0
    [Controller, gamma] = controllerConditioning(OpenLoopJFSystem, gamma, sdpVariables, opts, h);
end


end