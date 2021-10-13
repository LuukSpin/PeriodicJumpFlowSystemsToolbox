function [Controller, gamma] = JFMMHinfsyn(CLJFRefModel, OpenLoopJFSystem, h)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here

% Dimensions;
nref = size(CLJFRefModel.Ac, 1);
nx = size(OpenLoopJFSystem.Ac, 1);
nc = nx;
nu = size(OpenLoopJFSystem.Bud, 2);
ny = size(OpenLoopJFSystem.Cy, 1);

% LMI variables
Ph_ref = sdpvar(nref, nref, 'symmetric');
Y = sdpvar(nx, nx, 'symmetric');
X = sdpvar(nc, nc, 'symmetric');
Gamma = sdpvar(nx, nc, 'full');
Theta = sdpvar(nx, ny, 'full');
Upsilon = sdpvar(nu, nc, 'full');
Omega = sdpvar(nu, ny, 'full');

% Bisection settings
Nmax = 100; %Maximum numbers of iterations
tol = 1e-2; %Tolerance to calculate upperbound of Hinf norm

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
    [HinfLMIMatrix, A_bar, Ad, Bd, Cy] = fillHinfMMLMI(CLJFRefModel, OpenLoopJFSystem, Ph_ref, X, Y, h, gamma, Gamma, Theta, Upsilon, Omega);
    HinfLMI = HinfLMIMatrix >= 1e-9;
    
    
    % Add a constraint
    constraint = blkdiag(Ph_ref, [Y, eye(nx, nc); eye(nc, nx), X]) >= 1e-9;
    
    opts = LS.opts;
%     diagnostics = optimize(HinfLMI+constraint, [], opts.LMI)
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
Y = value(Y);
X = value(X);
Gamma = value(Gamma);
Theta = value(Theta);
Upsilon = value(Upsilon);
Omega = value(Omega);

U = X;
V = inv(X)-Y;

% Calculate controller
controllerMat = [V, Y*A_bar*Bd; zeros(nu, size(V, 2)), eye(nu)]\[Gamma-Y*A_bar*Ad*X, Theta; Upsilon, Omega]/[U', zeros(size(Y, 1), ny); Cy*X, eye(ny)];
Controller = minreal(ss(controllerMat(1:nc, 1:nc), controllerMat(1:nc, nc+1:end), controllerMat(nc+1:end, 1:nc), controllerMat(nc+1:end, nc+1:end), h));

end

