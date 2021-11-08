function [HinfLMIMatrix, A_bar, Q_bar, Z_bar, W_bar] = fillHinfLMI(OpenLoopSDSystem, sdpVariableStruct, h, gamma)

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
Z_bar = OpenLoopSDSystem.Ad;
Q_bar = OpenLoopSDSystem.Bud;
W_bar = OpenLoopSDSystem.Cy;
E_bar = OpenLoopSDSystem.Bwd;
R_bar = OpenLoopSDSystem.Dy_wd;
Cd_bar = OpenLoopSDSystem.Czd;
D_bar = OpenLoopSDSystem.Dzd_u;
Ddd_p = OpenLoopSDSystem.Dzd_wd;

% sdp variables
Y = sdpVariableStruct.Y;
X = sdpVariableStruct.X;
Gamma = sdpVariableStruct.Gamma;
Theta = sdpVariableStruct.Theta;
Upsilon = sdpVariableStruct.Upsilon;
Omega = sdpVariableStruct.Omega;

%Dimensions
nc = size(X, 1);
nwd = size(Ddd_p, 2);
nzd = size(Ddd_p, 1);

h_tmp = 1;
%Diagonal entries
entry11 = Y;
entry22 = X;
entry33 = gamma^2*eye(nwd)*h_tmp;
entry44 = eye(rb);
entry55 = Y;
entry66 = X;
entry77 = eye(rc);
entry88 = eye(nzd)/h_tmp;

%Lower triangular entries
entry21 = eye(nc);
entry31 = zeros(nwd, nc);
entry32 = zeros(nwd, nc);
entry41 = zeros(rb, nc);
entry42 = zeros(rb, nc);
entry43 = zeros(rb, nwd);
entry51 = Y*A_bar*Z_bar+Theta*W_bar;
entry52 = Gamma;
entry53 = Y*A_bar*E_bar+Theta*R_bar;
entry54 = Y*B_bar;
entry61 = A_bar*(Z_bar+Q_bar*Omega*W_bar);
entry62 = A_bar*(Z_bar*X+Q_bar*Upsilon);
entry63 = A_bar*(E_bar+Q_bar*Omega*R_bar);
entry64 = B_bar;
entry65 = eye(nc);
entry71 = C_bar*(Z_bar+Q_bar*Omega*W_bar);
entry72 = C_bar*(Z_bar*X+Q_bar*Upsilon);
entry73 = C_bar*(E_bar+Q_bar*Omega*R_bar);
entry74 = zeros(rc, rb);
entry75 = zeros(rc, nc);
entry76 = zeros(rc, nc);
entry81 = Cd_bar+D_bar*Omega*W_bar;
entry82 = Cd_bar*X+D_bar*Upsilon;
entry83 = Ddd_p+D_bar*Omega*R_bar;
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

end

