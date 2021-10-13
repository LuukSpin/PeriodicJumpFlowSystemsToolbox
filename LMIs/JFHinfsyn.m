function [Controller, gamma] = JFHinfsyn(OpenLoopJFSystem, h)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
%

backoff = 1.01;
numAcc = 1e-8;

% Dimensions;
nx = size(OpenLoopJFSystem.Ac, 1);
nc = nx;
nu = size(OpenLoopJFSystem.Bud, 2);
ny = size(OpenLoopJFSystem.Cy, 1);

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
        [HinfLMIMatrix, A_bar, Q_bar, Z_bar, W_bar] = fillHinfLMI(OpenLoopJFSystem, X, Y, h, gamma, Gamma, Theta, Upsilon, Omega);
        HinfLMI = (HinfLMIMatrix+HinfLMIMatrix')/2 >= numAcc*eye(size(HinfLMIMatrix));
        
        % Add a constraint
        constraint = [Y, eye(nx, nc); eye(nc, nx), X] >= numAcc*eye(size(blkdiag(Y, X)));
        
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
Y_value = value(Y);
X_value = value(X);
Gamma_value = value(Gamma);
Theta_value = value(Theta);
Upsilon_value = value(Upsilon);
Omega_value = value(Omega);

U = X_value;
V = inv(X_value)-Y_value;

% Calculate controller
controllerMat = [V, Y_value*A_bar*Q_bar; zeros(nu, size(V, 2)), eye(nu)]\[Gamma_value-Y_value*A_bar*Z_bar*X_value, Theta_value; Upsilon_value, Omega_value]/[U', zeros(size(Y, 1), ny); W_bar*X_value, eye(ny)];
Controller = minreal(ss(controllerMat(1:nc, 1:nc), controllerMat(1:nc, nc+1:end), controllerMat(nc+1:end, 1:nc), controllerMat(nc+1:end, nc+1:end), h));

K_zpk = zpk(Controller);
K_zeros = K_zpk.z{:};
K_poles = K_zpk.p{:};

nrUnstabPole = length(K_poles(abs(K_poles)>1+eps));
nrNonMinPhaseZero = length(K_zeros(abs(K_zeros)>1+eps));

if nrUnstabPole+nrNonMinPhaseZero>0
    Controller = controllerConditioning(OpenLoopJFSystem, gamma, X, Y, Gamma, Theta, Upsilon, Omega, backoff, numAcc, h);
end


end