function [PassiveLMIMatrix, A_bar] = fillPassiveLMI(OpenLoopSDSystem, sdpVariableStruct, h)

%Calculate closed-loop flow matrices from the open-loop Jump/Flow system
[Aflow_JF, Bflow_JF, Cflow_JF, Dflow_JF] = OpenLoopSDSystem.ClosedLoopFlowMatrices();

M3 = Cflow_JF';
M2 = Dflow_JF+Dflow_JF';

%Calculate Hamiltonian using the closed-loop flow matrices and determien A, B and C_hat
[A_hat, B_hat, C_hat] = Hamiltonian(Aflow_JF, Bflow_JF, Cflow_JF, Dflow_JF, 1, h);

% Determine rank of B_hat and C_hat
rb = size(B_hat, 2);
rc = size(C_hat, 1);

% Truncate matrices to fit into the LMI
nx = OpenLoopSDSystem.nx;
A_bar = A_hat(1:nx, 1:nx);
B_bar = B_hat(1:nx, :);
C_bar = C_hat(:, 1:nx);

% Use these matrices to comply with Mannes Dreef definition of matrices
% used in the LMI
Ad = OpenLoopSDSystem.Ad;
Bud = OpenLoopSDSystem.Bud;
Cy = OpenLoopSDSystem.Cy;
Bwd = OpenLoopSDSystem.Bwd;
Dy_wd = OpenLoopSDSystem.Dy_wd;
Czd = OpenLoopSDSystem.Czd;
Dzd_ud = OpenLoopSDSystem.Dzd_ud;
Dzd_wd = OpenLoopSDSystem.Dzd_wd;

% % sdp variables
% Y = sdpVariableStruct.Y;
% X = sdpVariableStruct.X;
% Gamma = sdpVariableStruct.Gamma;
% Theta = sdpVariableStruct.Theta;
% Upsilon = sdpVariableStruct.Upsilon;
% Omega = sdpVariableStruct.Omega;

Z1 = Czd+Dzd_ud*sdpVariableStruct.Omega*Cy;
Z2 = Czd*sdpVariableStruct.X+Dzd_ud*sdpVariableStruct.Upsilon;
Z3 = Dzd_wd+Dzd_ud*sdpVariableStruct.Omega*Dy_wd;

%Dimensions
nc = size(sdpVariableStruct.X, 1);
nd = size(Dzd_wd, 2);

h_tmp = 1;
%Diagonal entries
entry11 = sdpVariableStruct.Y;
entry22 = sdpVariableStruct.X;
entry33 = Z3+Z3';
entry44 = eye(rc);              %
entry55 = sdpVariableStruct.Y;  % 
entry66 = sdpVariableStruct.X;  %
entry77 = eye(rb);              %

%Lower triangular entries
entry21 = eye(nc);
entry31 = Z1;
entry32 = Z2;
entry41 = C_bar*(Ad+Bud*sdpVariableStruct.Omega*Cy);
entry42 = C_bar*(Ad*sdpVariableStruct.X+Bud*sdpVariableStruct.Upsilon);
entry43 = C_bar*(Bwd+Bud*sdpVariableStruct.Omega*Dy_wd);
entry51 = sdpVariableStruct.Y*A_bar*Ad+sdpVariableStruct.Theta*Cy;
entry52 = sdpVariableStruct.Gamma;
entry53 = sdpVariableStruct.Y*A_bar*Bwd+sdpVariableStruct.Theta*Dy_wd;
entry54 = zeros(nc, rc);
entry61 = A_bar*(Ad+Bud*sdpVariableStruct.Omega*Cy);
entry62 = A_bar*(Ad*sdpVariableStruct.X+Bud*sdpVariableStruct.Upsilon);
entry63 = A_bar*(Bwd+Bud*sdpVariableStruct.Omega*Dy_wd);
entry64 = zeros(nc, rc);
entry65 = eye(nc);
entry71 = zeros(rb, nc);
entry72 = zeros(rb, nc);
entry73 = zeros(rb, nwd);
entry74 = zeros(rb, rc);
entry75 = B_bar'*sdpVariableStruct.Y;
entry76 = B_bar';

PassiveLMIMatrix = [entry11, entry21', entry31', entry41', entry51', entry61', entry71';...
                 entry21, entry22, entry32', entry42', entry52', entry62', entry72';...
                 entry31, entry32, entry33, entry43', entry53', entry63', entry73';...
                 entry41, entry42, entry43, entry44, entry54', entry64', entry74';...
                 entry51, entry52, entry53, entry54, entry55, entry65', entry75';...
                 entry61, entry62, entry63, entry64, entry65, entry66, entry76';...
                 entry71, entry72, entry73, entry74, entry75, entry76, entry77];

PassiveLMIMatrix = (PassiveLMIMatrix+PassiveLMIMatrix')/2;
end

