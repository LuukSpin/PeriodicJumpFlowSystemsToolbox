function [HinfLMIMatrix, A_bar] = fillHinfLMIold(OpenLoopSDSystem, sdpVariableStruct, h, gamma)

%Calculate closed-loop flow matrices from the open-loop Jump/Flow system
[Aflow_JF, Bflow_JF, Cflow_JF, Dflow_JF] = OpenLoopSDSystem.ClosedLoopFlowMatrices();

%Calculate Hamiltonian using the closed-loop flow matrices and determien A, B and C_hat
[A_hat, B_hat, C_hat] = HamiltonianSD(Aflow_JF, Bflow_JF, Cflow_JF, Dflow_JF, gamma, h);

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

%Dimensions
nc = size(sdpVariableStruct.X, 1);
nwd = size(Dzd_wd, 2);
nzd = size(Dzd_wd, 1);

h_tmp = 1;
%Diagonal entries
entry11 = sdpVariableStruct.Y;
entry22 = sdpVariableStruct.X;
entry33 = gamma*eye(nwd)*h_tmp;
entry44 = eye(rb);
entry55 = sdpVariableStruct.Y;
entry66 = sdpVariableStruct.X;
entry77 = eye(rc);
entry88 = gamma*eye(nzd)/h_tmp;

%Lower triangular entries
entry21 = eye(nc);
entry31 = zeros(nwd, nc);
entry32 = zeros(nwd, nc);
entry41 = zeros(rb, nc);
entry42 = zeros(rb, nc);
entry43 = zeros(rb, nwd);
entry51 = sdpVariableStruct.Y*A_bar*Ad+sdpVariableStruct.Theta*Cy;
entry52 = sdpVariableStruct.Gamma;
entry53 = sdpVariableStruct.Y*A_bar*Bwd+sdpVariableStruct.Theta*Dy_wd;
entry54 = sdpVariableStruct.Y*B_bar;
entry61 = A_bar*(Ad+Bud*sdpVariableStruct.Omega*Cy);
entry62 = A_bar*(Ad*sdpVariableStruct.X+Bud*sdpVariableStruct.Upsilon);
entry63 = A_bar*(Bwd+Bud*sdpVariableStruct.Omega*Dy_wd);
entry64 = B_bar;
entry65 = eye(nc);
entry71 = C_bar*(Ad+Bud*sdpVariableStruct.Omega*Cy);
entry72 = C_bar*(Ad*sdpVariableStruct.X+Bud*sdpVariableStruct.Upsilon);
entry73 = C_bar*(Bwd+Bud*sdpVariableStruct.Omega*Dy_wd);
entry74 = zeros(rc, rb);
entry75 = zeros(rc, nc);
entry76 = zeros(rc, nc);
entry81 = Czd+Dzd_ud*sdpVariableStruct.Omega*Cy;
entry82 = Czd*sdpVariableStruct.X+Dzd_ud*sdpVariableStruct.Upsilon;
entry83 = Dzd_wd+Dzd_ud*sdpVariableStruct.Omega*Dy_wd;
entry84 = zeros(nzd, rb);
entry85 = zeros(nzd, nc);
entry86 = zeros(nzd, nc);
entry87 = zeros(nzd, rc);

HinfLMIMatrix = [entry11, entry21', entry31', entry41', entry51', entry61', entry71', entry81';...
                 entry21, entry22, entry32', entry42', entry52', entry62', entry72', entry82';...
                 entry31, entry32, entry33, entry43', entry53', entry63', entry73', entry83';...
                 entry41, entry42, entry43, entry44, entry54', entry64', entry74', entry84';...
                 entry51, entry52, entry53, entry54, entry55, entry65', entry75', entry85';...
                 entry61, entry62, entry63, entry64, entry65, entry66, entry76', entry86';...
                 entry71, entry72, entry73, entry74, entry75, entry76, entry77, entry87';...
                 entry81, entry82, entry83, entry84, entry85, entry86, entry87, entry88];

HinfLMIMatrix = (HinfLMIMatrix+HinfLMIMatrix')/2;
end

