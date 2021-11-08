function [Controller, gamValue] = controllerConstruction(OpenLoopSDSystem, A_bar, gamma, sdpVariableStruct, h, optsSD)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
%

arguments
    OpenLoopSDSystem    (1,1) OpenLoopSampledDataSystem
    A_bar               double
    gamma               (1,1) double
    sdpVariableStruct   struct
    h                   (1,1) double
    optsSD              SDopts = SDopts();
end

% Dimensions
ny = OpenLoopSDSystem.ny;
nu = OpenLoopSDSystem.nu;
nc = size(sdpVariableStruct.X, 1);

% Sampled-data system matrices
Ad = OpenLoopSDSystem.Ad;
Bud = OpenLoopSDSystem.Bud;
Cy = OpenLoopSDSystem.Cy;

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
controllerMat = [V, Y_value*A_bar*Ad; zeros(nu, size(V, 2)), eye(nu)]\[Gamma_value-Y_value*A_bar*Bud*X_value, Theta_value; Upsilon_value, Omega_value]/[U', zeros(size(Y_value, 1), ny); Cy*X_value, eye(ny)];
Controller = minreal(ss(controllerMat(1:nc, 1:nc), controllerMat(1:nc, nc+1:end), controllerMat(nc+1:end, 1:nc), controllerMat(nc+1:end, nc+1:end), h), [], false);

K_zpk = zpk(Controller);
K_zeros = K_zpk.z{:};
K_poles = K_zpk.p{:};

nrUnstabPole = length(K_poles(abs(K_poles)>1+eps));
nrNonMinPhaseZero = length(K_zeros(abs(K_zeros)>1+eps));

if nrUnstabPole+nrNonMinPhaseZero>0
    [Controller, gamValue] = controllerConditioning(OpenLoopSDSystem, gamma, sdpVariableStruct, h, optsSD);
end

end