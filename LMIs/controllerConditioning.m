function [Controller, gamValue] = controllerConditioning(OpenLoopSDSystem, gamma, sdpVariableStruct, h, optsSD)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
%

arguments
    OpenLoopSDSystem    (1,1) OpenLoopSampledDataSystem
    gamma               (1,1) double
    sdpVariableStruct   struct
    h                   (1,1) double
    optsSD              SDopts = SDopts();
end

backoff = optsSD.LMI.backoffFactor;
numAcc = optsSD.LMI.numericalAccuracy;
alphabackoff = 25;
wellConditioned = false;
maxIt = ceil(log(2)/log(backoff));
counter = 0;

while ~wellConditioned && (counter < maxIt)
    
    % Dimensions;
    nu = size(OpenLoopSDSystem.Bud, 2);
    ny = size(OpenLoopSDSystem.Cy, 1);
    nx = size(sdpVariableStruct.Y, 1);
    nc = size(sdpVariableStruct.X, 1);
    opts = LS.opts;
    
    LMI = [];
    
    gamValue = gamma*backoff;
    [HinfLMI, A_bar] = fillHinfLMI(OpenLoopSDSystem, sdpVariableStruct, h, gamValue);
    LMI = [LMI; (HinfLMI+HinfLMI')/2 >= numAcc*eye(size(HinfLMI))];
    posDefConstraint = [sdpVariableStruct.Y, eye(nx, nc); eye(nc, nx), sdpVariableStruct.X];
    LMI = [LMI; posDefConstraint >= numAcc*eye(size(posDefConstraint))];
    
    % Add numerical aspect contraints
    alpha = sdpvar(1,1);
    NumericalConstraint1 = (sdpVariableStruct.X - alpha*eye(size(sdpVariableStruct.X)) <= -numAcc*eye(size(sdpVariableStruct.X)));
    NumericalConstraint2 = (sdpVariableStruct.Y - alpha*eye(size(sdpVariableStruct.Y)) <= -numAcc*eye(size(sdpVariableStruct.X)));
    ParamBlock = [sdpVariableStruct.Gamma, sdpVariableStruct.Theta; sdpVariableStruct.Upsilon, sdpVariableStruct.Omega];
    alphaBlock = alpha*eye(size(ParamBlock));
    NumericalConstraint3 = [alphaBlock, ParamBlock; ParamBlock', alphaBlock] >= numAcc*eye(size(blkdiag(alphaBlock, alphaBlock)));
    
    NumericalConstraints = [NumericalConstraint1; NumericalConstraint2; NumericalConstraint3];
    
    LMI = [LMI; NumericalConstraints];
    
    diagnostics = optimize(LMI, alpha, opts.LMI);
    
    alphaValue = value(alpha)*alphabackoff;
    LMI = replace(LMI, alpha, alphaValue);
    diagnostics = optimize(LMI, [], opts.LMI);
    
    beta = sdpvar(1,1); assign(beta, 1);
    newPosDefConstraint = [sdpVariableStruct.Y, eye(nx, nc)*beta; eye(nc, nx)*beta, sdpVariableStruct.X];
    
    LMI = [LMI; newPosDefConstraint >= numAcc*eye(size(newPosDefConstraint)); beta >= 1];
    diagnostics = optimize(LMI, -beta, opts.LMI);
    betaValue = value(beta);%/backoff;
    
    LMI = replace(LMI, beta, betaValue);
    diagnostics = optimize(LMI, [], opts.LMI);

    Controller = controllerConstruction(OpenLoopSDSystem, A_bar, sdpVariableStruct, h);
    
    K_zpk = zpk(Controller);
    K_zeros = K_zpk.z{:};
    K_poles = K_zpk.p{:};
    
    nrUnstabPole = length(K_poles(abs(K_poles)>1+eps));
    nrNonMinPhaseZero = length(K_zeros(abs(K_zeros)>1+eps));
    
    if (nrUnstabPole+nrNonMinPhaseZero) ~= 0
        backoff = backoff*optsSD.LMI.backoffFactor;
    else
        wellConditioned = true;
    end
    counter = counter+1;
end

if ~wellConditioned
    warning('The controller is ill-conditioned and may have poles and/or zeros outside the unit circle');
end

end