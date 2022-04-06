function [HinfLMIAnalysisMatrix, B_hat] = fillPassiveAnalysisLMI(objCLJF, Ph, opts)

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
[A_hat, B_hat, C_hat] = Hamiltonian(Ac, Bc, Cc, Dc, opts);

%Dimensions
nx = objCLJF.nx;
nzd = objCLJF.nzd;
rb = size(B_hat, 2);
rc = size(C_hat, 1);


%Diagonal entries
entry11 = Ph;
entry22 = Dd+Dd';
entry33 = eye(rc);
entry44 = Ph;
entry55 = eye(rb);

%Lower triangular entries
entry21 = Cd;
entry31 = C_hat*Ad;
entry32 = C_hat*Bd;
entry41 = Ph*A_hat*Ad;
entry42 = Ph*A_hat*Bd;
entry43 = zeros(nx, rc);
entry51 = zeros(rb, nx);
entry52 = zeros(rb, nzd);
entry53 = zeros(rb, rc);
entry54 = B_hat'*Ph;

HinfLMIAnalysisMatrix = [entry11, entry21', entry31', entry41', entry51';...
                         entry21, entry22, entry32', entry42', entry52';...
                         entry31, entry32, entry33, entry43', entry53';...
                         entry41, entry42, entry43, entry44, entry54';...
                         entry51, entry52, entry53, entry54, entry55];

end