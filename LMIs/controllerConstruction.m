function Controller = controllerConstruction(OpenLoopSDSystem, A_bar, sdpVariableStruct, opts)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
%

arguments
    OpenLoopSDSystem        (1,1) OpenLoopSampledDataSystem
    A_bar                   double
    sdpVariableStruct       struct
    opts                       (1,1) SDopts
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
Y_value = value(sdpVariableStruct.Y);
X_value = value(sdpVariableStruct.X);
Gamma_value = value(sdpVariableStruct.Gamma);
Theta_value = value(sdpVariableStruct.Theta);
Upsilon_value = value(sdpVariableStruct.Upsilon);
Omega_value = value(sdpVariableStruct.Omega);

U = X_value;
V = (inv(X_value)-Y_value)';

% Calculate controller
controllerMat = [V, Y_value*A_bar*Bud; zeros(nu, size(V, 2)), eye(nu)]\[Gamma_value-Y_value*A_bar*Ad*X_value, Theta_value; Upsilon_value, Omega_value]/[U', zeros(size(Y_value, 1), ny); Cy*X_value, eye(ny)];
Controller = minreal(ss(controllerMat(1:nc, 1:nc), controllerMat(1:nc, nc+1:end), controllerMat(nc+1:end, 1:nc), controllerMat(nc+1:end, nc+1:end), opts.simulation.SampleTime), [], false);

end