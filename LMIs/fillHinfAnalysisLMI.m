function HinfLMIAnalysisMatrix = fillHinfAnalysisLMI(objCLJF, Ph, h, gamma)

% Flow matrices
Ac = objCLJF.Ac;
Bc = objCLJF.Bwc;
Cc = objCLJF.Czc;
Dc = objCLJF.Dzc_wc;

% Jump matrices
Ad = objCLJF.Ad;
Bd = objCLJF.Bwd;
Cd = objCLJF.Czd;
Dd = objCLJF.Dzd_wd;

%Calculate Hamiltonian using the closed-loop flow matrices and determien A, B and C_hat
[A_hat, B_hat, C_hat] = HamiltonianJF(Ac, Bc, Cc, Dc, gamma, h);

%Dimensions
nx = size(objCLJF.Ac,1);
nwd = size(objCLJF.Dzd_wd, 2);
nzd = size(objCLJF.Dzd_wd, 1);
nb = size(B_hat, 2);
nc = size(C_hat, 1);

htmp = 1;
%Diagonal entries
entry11 = Ph;
entry22 = gamma^2*eye(nwd)*htmp;
entry33 = eye(nb);
entry44 = Ph;
entry55 = eye(nc);
entry66 = eye(nzd)/htmp;

%Lower triangular entries
entry21 = zeros(nwd, nx);
entry31 = zeros(nb, nx);
entry32 = zeros(nb, nwd);
entry41 = Ph*A_hat*Ad;
entry42 = Ph*A_hat*Bd;
entry43 = Ph*B_hat;
entry51 = C_hat*Ad;
entry52 = C_hat*Bd;
entry53 = zeros(nc, nb);
entry54 = zeros(nc, nx);
entry61 = Cd;
entry62 = Dd;
entry63 = zeros(nzd, nb);
entry64 = zeros(nzd, nx);
entry65 = zeros(nzd, nc);

HinfLMIAnalysisMatrix = [entry11, entry21', entry31', entry41', entry51', entry61';...
                         entry21, entry22, entry32', entry42', entry52', entry62';...
                         entry31, entry32, entry33, entry43', entry53', entry63';...
                         entry41, entry42, entry43, entry44, entry54', entry64';...
                         entry51, entry52, entry53, entry54, entry55, entry65';...
                         entry61, entry62, entry63, entry64, entry65, entry66];

end