function LMI = fillAnalysisLMI(sys, opts)

arguments
    sys       (1,1) {mustBeA(sys, ["JumpFlowSystem", "ss", "zpk", "tf"])}
    opts      (1,1) jfopt
end

%Dimensions
nx = sys.nx;
nwd = sys.nwd;
nzd = sys.nzd;

% Check all specified system norms such as Hinf, H2, H2g, L1
switch lower(opts.performanceString)
    case {'hinf', 'l2', 'h-inf'}
        Qd = opts.performanceValue*eye(sys.nwd);
        Sd = zeros(sys.nwd, sys.nzd);
        Rd = -opts.performanceValue\eye(sys.nzd);
    case {'h2'}
        error('H2-norm is not yet implemented');
    case {'h2g'}
        error('genH2-norm is not yet implemented');
    case {'l1'}
        error('L1-norm is not yet implemented');
    case {'passivity', 'passive', 'pass'}
        Qd = zeros(sys.nwd);
        Sd = -eye(sys.nwd, sys.nzd)/2;
        Rd = zeros(sys.nzd);
    case {'qrs', 'quad', 'quadratic'}
        Qd = opts.performanceValue.Qd;
        if ~all(Qd==Qd', 'all')
            error('Continuos performance matrix Qd must be symmetric');
        end
        Sd = opts.performanceValue.Sd;
        Rd = opts.performanceValue.Rd;
        if ~all(Rd==Rd', 'all')
            error('Continuos performance matrix Rd must be symmetric');
        elseif any(eig(Rd)>=0)
            if any(eig(Rd) > 0)
                error('Discrete performance matrix Rd must be negative semi-definite');
            elseif any(eig(Rd) == 0)
                if ~all(eig(Rd)==0)
                    error('Discrete performance matrix Rd must be negative definite or the zero matrix');
                end
            end
        end
end

%Calculate Hamiltonian using the closed-loop flow matrices and determien A, B and C_hat
[A_hat, B_hat, C_hat] = jfhamilexp(sys, opts);
Ad = sys.Ad;
Bd = sys.Bwd;
Cd = sys.Czd;
Dd = sys.Dzd_wd;

rb = size(B_hat, 2);
rc = size(C_hat, 1);

Ph = sdpvar(nx, nx, 'sym');

% 6 --> 3 --> 6
% 4 --> 5 --> 4

% htmp = 1;
% entry22 = gamma*eye(nwd)*htmp;
% entry33 = gamma*eye(nzd)/htmp;

if all(eig(Rd)==0)

    %Diagonal entries
    entry11 = Ph;
    entry22 = Qd+Sd*Dd+Dd'*Sd';
    entry33 = eye(rc);
    entry44 = Ph;
    entry55 = eye(rb);

    %Lower triangular entries
    entry21 = Sd*Cd;
    entry31 = C_hat*Ad;
    entry32 = C_hat*Bd;
    entry41 = Ph*A_hat*Ad;
    entry42 = Ph*A_hat*Bd;
    entry43 = zeros(nx, rc);
    entry51 = zeros(rb, nx);
    entry52 = zeros(rb, nwd);
    entry53 = zeros(rb, rc);
    entry54 = B_hat'*Ph;

    jfAnalysisMatrix = [entry11, entry21', entry31', entry41', entry51';...
                             entry21, entry22, entry32', entry42', entry52';...
                             entry31, entry32, entry33, entry43', entry53';...
                             entry41, entry42, entry43, entry44, entry54';...
                             entry51, entry52, entry53, entry54, entry55];
else

    %Diagonal entries
    entry11 = Ph;
    entry22 = Qd+Sd*Dd+Dd'*Sd';
    entry33 = -inv(Rd);
    entry44 = eye(rc);
    entry55 = Ph;
    entry66 = eye(rb);

    %Lower triangular entries
    entry21 = Sd*Cd;
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

    jfAnalysisMatrix = [entry11, entry21', entry31', entry41', entry51', entry61';...
                             entry21, entry22, entry32', entry42', entry52', entry62';...
                             entry31, entry32, entry33, entry43', entry53', entry63';...
                             entry41, entry42, entry43, entry44, entry54', entry64';...
                             entry51, entry52, entry53, entry54, entry55, entry65';...
                             entry61, entry62, entry63, entry64, entry65, entry66];
end

numAcc = opts.LMI.numericalAccuracy;
LMI = jfAnalysisMatrix >= numAcc*eye(size(jfAnalysisMatrix));

end