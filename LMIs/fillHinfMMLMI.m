function [HinfLMIMatrix, A_bar, Ad, Bd, Cy] = fillHinfMMLMI(CLJFRefModel, OpenLoopJFSystem, Ph_ref, X, Y, h, gamma, Gamma, Theta, Upsilon, Omega)

%Dimensions
nref = size(Ph_ref, 1);
nx = size(Y, 1);
nc = size(X, 1);
nwc = size(CLJFRefModel.Dzc_wc, 2);
nzc = size(CLJFRefModel.Dzc_wc, 1);
nwd = size(CLJFRefModel.Dzd_wd, 2);
nzd = size(CLJFRefModel.Dzd_wd, 1);

JFErrorSystem = makeErrorJFSystem(CLJFRefModel, OpenLoopJFSystem);
% JFErrorSystem = appendSDWeightingFilters(JFErrorSystem, ss(eye(nwc)), ss(eye(nzc))*1e5, c2d(ss(eye(nwd)), h, 'tustin')*1e5, c2d(ss(eye(nzd)), h, 'tustin'));
% scaling factor to emphasize importance of small error signal
SF = 1;
JFErrorSystem.Czc = JFErrorSystem.Czc*SF;
JFErrorSystem.Dzc_wc = JFErrorSystem.Dzc_wc*SF;
JFErrorSystem.Czd = JFErrorSystem.Czd*SF;
JFErrorSystem.Dzd_wd = JFErrorSystem.Dzd_wd*SF;
JFErrorSystem.Dzd_u = JFErrorSystem.Dzd_u*SF;

%Calculate closed-loop flow matrices from the open-loop Jump/Flow system
[Aflow_JF, Bflow_JF, Cflow_JF, Dflow_JF] = closedLoopFlowMatrices(JFErrorSystem, nc);

%Calculate Hamiltonian using the closed-loop flow matrices and determien A, B and C_hat
[A_hat, B_hat, C_hat] = HamiltonianJF(Aflow_JF, Bflow_JF, Cflow_JF, Dflow_JF, gamma, h);

% Determine rank of B_hat and C_hat
rb = size(B_hat, 2);
rc = size(C_hat, 1);

% Truncate matrices to fit into the LMI
Abar_ref = A_hat(1:nref, 1:nref);
A_bar = A_hat(nref+1:nref+nx, nref+1:nref+nx);
Bbar_ref = B_hat(1:nref, :);
B_bar = B_hat(nref+1:nref+nx, :);
Cbar_ref = C_hat(:, 1:nref);
C_bar = C_hat(:, nref+1:nref+nx);

% Reference model jump matrices
Ad_ref = CLJFRefModel.Ad;
Bd_ref = CLJFRefModel.Bwd;
Cd_ref = CLJFRefModel.Czd;
Dd_ref = CLJFRefModel.Dzd_wd;

% OL jump matrices
Ad = OpenLoopJFSystem.Ad;
Bd = OpenLoopJFSystem.Bwd;
Bu = OpenLoopJFSystem.Bud;
Cy = OpenLoopJFSystem.Cy;
Cd = OpenLoopJFSystem.Czd;
Ddd = OpenLoopJFSystem.Dzd_wd;
Ddu = OpenLoopJFSystem.Dzd_u;
Dyd = OpenLoopJFSystem.Dy_wd;

%Diagonal entries
entry11 = Ph_ref;
entry22 = Y;
entry33 = X;
entry44 = gamma^2*eye(nwd);
entry55 = eye(rb);
entry66 = Ph_ref;
entry77 = Y;
entry88 = X;
entry99 = eye(rc);
entry1010 = eye(nzd);

%Lower triangular entries
entry21 = eye(nx, nref)/100;
entry31 = zeros(nc, nref);
entry32 = eye(nc, nx);
entry41 = zeros(nwd, nref);
entry42 = zeros(nwd, nx);
entry43 = zeros(nwd, nc);
entry51 = zeros(rb, nref);
entry52 = zeros(rb, nx);
entry53 = zeros(rb, nc);
entry54 = zeros(rb, nwd);
entry61 = Ph_ref*Abar_ref*Ad_ref;
entry62 = zeros(nref, nx);
entry63 = zeros(nref, nc);
entry64 = Ph_ref*Abar_ref*Bd_ref;
entry65 = Ph_ref*Bbar_ref;
entry71 = zeros(nx, nref);
entry72 = Y*A_bar*Ad+Theta*Cy;
entry73 = Gamma;
entry74 = Y*A_bar*Bd+Theta*Dyd;
entry75 = Y*B_bar;
entry76 = zeros(nx, nref);
entry81 = zeros(nc, nref);
entry82 = A_bar*(Ad+Bu*Omega*Cy);
entry83 = A_bar*(Ad*X+Bu*Upsilon);
entry84 = A_bar*(Bd+Bu*Omega*Dyd);
entry85 = B_bar;
entry86 = zeros(nc, nref);
entry87 = eye(nc, nx);
entry91 = Cbar_ref*Ad_ref;
entry92 = C_bar*(Ad+Bu*Omega*Cy);
entry93 = C_bar*(Ad*X+Bu*Upsilon);
entry94 = Cbar_ref*Bd_ref+C_bar*(Bd+Bu*Omega*Dyd);
entry95 = zeros(rc, rb);
entry96 = zeros(rc, nref);
entry97 = zeros(rc, nx);
entry98 = zeros(rc, nc);
entry101 = -Cd_ref;
entry102 = Cd+Ddu*Omega*Cy;
entry103 = Cd*X+Ddu*Upsilon;
entry104 = Ddd+Ddu*Omega*Dyd-Dd_ref;
entry105 = zeros(nzd, rb);
entry106 = zeros(nzd, nref);
entry107 = zeros(nzd, nc);
entry108 = zeros(nzd, nc);
entry109 = zeros(nzd, rc);

HinfLMIMatrix = [entry11, entry21', entry31', entry41', entry51', entry61', entry71', entry81', entry91', entry101';...
                 entry21, entry22, entry32', entry42', entry52', entry62', entry72', entry82', entry92', entry102';...
                 entry31, entry32, entry33, entry43', entry53', entry63', entry73', entry83', entry93', entry103';...
                 entry41, entry42, entry43, entry44, entry54', entry64', entry74', entry84', entry94', entry104';...
                 entry51, entry52, entry53, entry54, entry55, entry65', entry75', entry85', entry95', entry105';...
                 entry61, entry62, entry63, entry64, entry65, entry66, entry76', entry86', entry96', entry106';...
                 entry71, entry72, entry73, entry74, entry75, entry76, entry77, entry87', entry97', entry107';...
                 entry81, entry82, entry83, entry84, entry85, entry86, entry87, entry88, entry98', entry108';...
                 entry91, entry92, entry93, entry94, entry95, entry96, entry97, entry98, entry99, entry109';...
                 entry101, entry102, entry103, entry104, entry105, entry106, entry107, entry108, entry109, entry1010];

end

