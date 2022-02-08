function [sdpVariables, A_bar] = controllerConditioning(OpenLoopSDSystem, gamma, sdpVariableStruct, h, performanceIndicator, opts)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
%

arguments
    OpenLoopSDSystem        (1,1) OpenLoopSampledDataSystem
    gamma                   (1,1) double
    sdpVariableStruct       struct
    h                       (1,1) double
    performanceIndicator    string = 'Hinf'
    opts                    SDopts = SDopts()
end

backoff = opts.LMI.backoffFactor;
numAcc = opts.LMI.numericalAccuracy;
solverSettings = opts.LMI.solverOptions';
alphabackoff = 25;

% Dimensions;
nu = size(OpenLoopSDSystem.Bud, 2);
ny = size(OpenLoopSDSystem.Cy, 1);
nx = size(sdpVariableStruct.Y, 1);
nc = size(sdpVariableStruct.X, 1);

gamValue = gamma*backoff;

if strcmpi(performanceIndicator, 'Hinf')
    [synthesisMatrix, A_bar] = fillHinfLMI(OpenLoopSDSystem, sdpVariableStruct, h, gamValue);
else
    error('Only "Hinf" controller synthesis is specified at the moment.');
end

LMI = synthesisMatrix >= numAcc*eye(size(synthesisMatrix));

% Add numerical aspect contraints
alpha = sdpvar(1,1);
NumericalConstraint1 = (sdpVariableStruct.X - alpha*eye(size(sdpVariableStruct.X)) <= -numAcc*eye(size(sdpVariableStruct.X)));
NumericalConstraint2 = (sdpVariableStruct.Y - alpha*eye(size(sdpVariableStruct.Y)) <= -numAcc*eye(size(sdpVariableStruct.Y)));
ParamBlock = [sdpVariableStruct.Gamma, sdpVariableStruct.Theta; sdpVariableStruct.Upsilon, sdpVariableStruct.Omega];
alphaBlock = alpha*eye(size(ParamBlock));
NumericalConstraint3 = [alphaBlock, ParamBlock; ParamBlock', alphaBlock] >= numAcc*eye(size(blkdiag(alphaBlock, alphaBlock)));

NumericalConstraints = [NumericalConstraint1, NumericalConstraint2, NumericalConstraint3];

LMI = [LMI, NumericalConstraints];

diagnostics = optimize(LMI, alpha, solverSettings);

alphaValue = value(alpha)*alphabackoff;
LMI = replace(LMI, alpha, alphaValue);

beta = sdpvar(1,1); assign(beta, 1);
newPosDefConstraint = [sdpVariableStruct.Y, eye(nx, nc)*beta; eye(nc, nx)*beta, sdpVariableStruct.X];

LMI = [LMI, newPosDefConstraint >= numAcc*eye(size(newPosDefConstraint)), beta >= 1];
diagnostics = optimize(LMI, -beta, solverSettings);

sdpVariables = sdpVariableStruct;

end