function [K, val] = controllerConditioning(sys, sdpVars, opts)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
%

arguments
    sys                     OpenLoopSampledDataSystem
    sdpVars                 struct
    opts                    jfopt
end

backoff = opts.LMI.backoffFactor;
numAcc = opts.LMI.numericalAccuracy;
solverSettings = opts.LMI.solverOptions;
alphabackoff = 25;
maxIter = opts.LMI.bisection.maxIter;

% Dimensions;
nx = size(sdpVars.Y, 1);
nc = size(sdpVars.X, 1);

switch lower(opts.performanceString)
    case{'hinf', 'l2', 'h00', 'hoo'}
        opts.performanceValue = opts.performanceValue*backoff;
end

LMI = fillDissSynthesisLMI(sys, sdpVars, opts);

% Add numerical contraints on lyapunov and controller construction matrices
alpha = sdpvar(1,1);
lyapNumConstr1 = (sdpVars.X - alpha*eye(size(sdpVars.X)) <= -numAcc*eye(size(sdpVars.X)));
lyapNumConstr2 = (sdpVars.Y - alpha*eye(size(sdpVars.Y)) <= -numAcc*eye(size(sdpVars.Y)));
ParamBlock = [sdpVars.Gamma, sdpVars.Theta; sdpVars.Upsilon, sdpVars.Omega];
alphaBlock1 = alpha*eye(size(ParamBlock, 1));
alphaBlock2 = alpha*eye(size(ParamBlock, 2));
contrNumConstr = [alphaBlock1, ParamBlock; ParamBlock', alphaBlock2] >= numAcc*eye(size(blkdiag(alphaBlock1, alphaBlock2)));

NumericalConstraints = [lyapNumConstr1, lyapNumConstr2, contrNumConstr];

LMI = [LMI, NumericalConstraints];

diagnostics = optimize(LMI, alpha, solverSettings);

if diagnostics.problem ~= 0
    alphaValue = value(alpha)*alphabackoff;
    LMI = replace(LMI, alpha, alphaValue);
    diagnostics = optimize(LMI, [], solverSettings);
    if diagnostics.problem ~= 0
        error('Minimizing alpha in controllerConditioning leads to non-solvable LMI');
    end
end

beta = sdpvar(1,1);
assign(beta, 1); % make sure that solver maximizes beta starting from one
newPosDefConstraint = [sdpVars.Y, eye(nx, nc)*beta; eye(nc, nx)*beta, sdpVars.X];

LMI = [LMI, newPosDefConstraint >= numAcc*eye(size(newPosDefConstraint)), beta >= 1];
diagnostics = optimize(LMI, -beta, solverSettings);
betaValue = value(beta);

betaWhileLoop = diagnostics.problem ~= 0;

betaLoopCount = 0;
while betaWhileLoop
    betaValue = max([sqrt(value(beta)), betaValue/backoff]);
    LMI_new = replace(LMI, beta, betaValue);
    diagnostics = optimize(LMI_new, [], solverSettings);
    if diagnostics.problem == 0
        betaWhileLoop = false;
    end
    betaLoopCount = betaLoopCount+1;
    if betaLoopCount > maxIter
        error('Maximizing beta in controllerConditioning leads to non-solvable LMI');
    end
end

K = controllerConstruction(sys, sdpVars, opts);
val = opts.performanceValue;

if ~wellConditionedController(K)
    warning('Numerical conditioning of the controller still produces ill-conditioned controller.');
end
end