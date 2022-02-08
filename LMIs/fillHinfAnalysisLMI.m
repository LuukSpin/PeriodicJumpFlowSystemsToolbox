function [HinfLMIAnalysisMatrix, B_hat] = fillHinfAnalysisLMI(objCLJF, Ph, h, gamma)

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
nx = objCLJF.nx;
nwd = objCLJF.nwd;
nzd = objCLJF.nzd;
rb = size(B_hat, 2);
rc = size(C_hat, 1);

% 6 --> 3 --> 6
% 4 --> 5 --> 4

htmp = 1;
%Diagonal entries
entry11 = Ph;
entry22 = gamma*eye(nwd)*htmp;
entry33 = gamma*eye(nzd)/htmp;
entry44 = eye(rc);
entry55 = Ph;
entry66 = eye(rb);

%Lower triangular entries
entry21 = zeros(nwd, nx);
entry31 = Cd;
entry32 = Dd;
entry41 = C_hat*Ad;
entry42 = C_hat*Bd;
entry43 = zeros(rc, nzd);
entry51 = Ph*A_hat*Ad;
entry52 = Ph*A_hat*Bd;
entry53 = zeros(nx, nzd);
entry54 = zeros(nx, rc);
entry61 = zeros(rb, nx);
entry62 = zeros(rb, nwd);
entry63 = zeros(rb, nzd);
entry64 = zeros(rb, rc);
entry65 = B_hat'*Ph;

HinfLMIAnalysisMatrix = [entry11, entry21', entry31', entry41', entry51', entry61';...
                         entry21, entry22, entry32', entry42', entry52', entry62';...
                         entry31, entry32, entry33, entry43', entry53', entry63';...
                         entry41, entry42, entry43, entry44, entry54', entry64';...
                         entry51, entry52, entry53, entry54, entry55, entry65';...
                         entry61, entry62, entry63, entry64, entry65, entry66];

end