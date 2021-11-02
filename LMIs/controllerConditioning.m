function [Controller, gamValue] = controllerConditioning(OpenLoopJFSystem, gamma, X, Y, Gamma, Theta, Upsilon, Omega, backoff, numAcc, h)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
%
% Make sure that gamma is a number not a sdp-variable
if isa(gamma, 'sdpvar')
    gamma = value(gamma);
end

alphabackoff = 25;
wellConditioned = false;
maxIt = ceil(log(2)/log(backoff));
counter = 0;

while ~wellConditioned && (counter < maxIt)
    
    % Dimensions;
    nu = size(OpenLoopJFSystem.Bud, 2);
    ny = size(OpenLoopJFSystem.Cy, 1);
    nx = size(Y, 1);
    nc = size(X, 1);
    opts = LS.opts;
    
    LMI = [];
    
    gamValue = gamma*backoff;
    [HinfLMI, A_bar, Q_bar, Z_bar, W_bar] = fillHinfLMI(OpenLoopJFSystem, X, Y, h, gamValue, Gamma, Theta, Upsilon, Omega);
    LMI = [LMI; (HinfLMI+HinfLMI')/2 >= numAcc*eye(size(HinfLMI))];
    posDefConstraint = [Y, eye(nx, nc); eye(nc, nx), X];
    LMI = [LMI; posDefConstraint >= numAcc*eye(size(posDefConstraint))];
    
    % Add numerical aspect contraints
    alpha = sdpvar(1,1);
    NumericalConstraint1 = (X - alpha*eye(size(X)) <= -numAcc*eye(size(X)));
    NumericalConstraint2 = (Y - alpha*eye(size(Y)) <= -numAcc*eye(size(X)));
    ParamBlock = [Gamma, Theta; Upsilon, Omega];
    alphaBlock = alpha*eye(size(ParamBlock));
    NumericalConstraint3 = [alphaBlock, ParamBlock; ParamBlock', alphaBlock] >= numAcc*eye(size(blkdiag(alphaBlock, alphaBlock)));
    
    NumericalConstraints = [NumericalConstraint1; NumericalConstraint2; NumericalConstraint3];
    
    LMI = [LMI; NumericalConstraints];
    
    diagnostics = optimize(LMI, alpha, opts.LMI);
    
    alphaValue = value(alpha)*20;
    LMI = replace(LMI, alpha, alphaValue);
    diagnostics = optimize(LMI, [], opts.LMI);
    
    beta = sdpvar(1,1); assign(beta, 1);
    newPosDefConstraint = [Y, eye(nx, nc)*beta; eye(nc, nx)*beta, X];
    
    LMI = [LMI; newPosDefConstraint >= numAcc*eye(size(newPosDefConstraint)); beta >= 1];
    diagnostics = optimize(LMI, -beta, opts.LMI);
    betaValue = value(beta);%/backoff;
    
    LMI = replace(LMI, beta, betaValue);
    diagnostics = optimize(LMI, [], opts.LMI);
    
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
    controllerMat = [V, Y_value*A_bar*Q_bar; zeros(nu, size(V, 2)), eye(nu)]\[Gamma_value-Y_value*A_bar*Z_bar*X_value, Theta_value; Upsilon_value, Omega_value]/[U', zeros(size(Y_value, 1), ny); W_bar*X_value, eye(ny)];
    Controller = minreal(ss(controllerMat(1:nc, 1:nc), controllerMat(1:nc, nc+1:end), controllerMat(nc+1:end, 1:nc), controllerMat(nc+1:end, nc+1:end), h), [], false);
    
    K_zpk = zpk(Controller);
    K_zeros = K_zpk.z{:};
    K_poles = K_zpk.p{:};
    
    nrUnstabPole = length(K_poles(abs(K_poles)>1+eps));
    nrNonMinPhaseZero = length(K_zeros(abs(K_zeros)>1+eps));
    
    if (nrUnstabPole+nrNonMinPhaseZero) ~= 0
        backoff = backoff*1.01;
    else
        wellConditioned = true;
    end
    counter = counter+1;
end

if ~wellConditioned
    warning('The controller is ill-conditioned and may have poles and/or zeros outside the unit circle');
end

end