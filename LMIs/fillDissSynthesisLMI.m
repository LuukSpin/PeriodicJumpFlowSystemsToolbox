function LMI = fillDissSynthesisLMI(sys, opts)

arguments
    sys       {mustBeA(sys, "OpenLoopSampledDataSystem")}%, "ss", "zpk", "tf"])}
    opts      jfopt
end

% for sampled-data systems make sure that a reconstructor is applied.
if strcmpi(sys.reconstructor, 'unspecified')
    sys = sys.applyReconstructor(opts);
end

%Dimensions
nx = sys.nx;
nc = nx;
nwd = sys.nwd;
nzd = sys.nzd;

% Check all specified system norms such as Hinf, H2, H2g, L1
switch lower(opts.performanceString)
    case {'hinf', 'l2', 'h-inf', 'h00', 'hoo'}
        Qd = opts.performanceValue*eye(sys.nwd);
        Sd = zeros(sys.nwd, sys.nzd);
        Rd = -opts.performanceValue\eye(sys.nzd);
    case {'h2g'}
        error('genH2-norm is not yet implemented');
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
A_bar = A_hat(1:nx, 1:nx);
B_bar = B_hat(1:nx, :);
C_bar = C_hat(:, 1:nx);

Ad = OpenLoopSDSystem.Ad;
Bd = OpenLoopSDSystem.Bwd;
Bu = OpenLoopSDSystem.Bud;
Cd = OpenLoopSDSystem.Czd;
Cy = OpenLoopSDSystem.Cy;
Dyd = OpenLoopSDSystem.Dy_wd;
Ddu = OpenLoopSDSystem.Dzd_ud;
Ddd = OpenLoopSDSystem.Dzd_wd;
%Dyu is not necessary since it may be assumed to be zero w.l.o.g.
%If Dyu is in fact not zero, this will be compensated for in the controller
%construction.

rb = size(B_hat, 2);
rc = size(C_hat, 1);

Omega = [Ad, Bd; Cd, Ddd]+[Bu; Ddu]*sdpVars.Omega*[Cy, Dyd];
Omega_a = Omega(1:nx, 1:nx);
Omega_b = Omega(1:nx, nx+1:end);
Omega_c = Omega(nx+1:end, 1:nx);
Omega_d = Omega(nx+1:end, nx+1:end);

Z = Qd+Sd*Omega_d+Omega_d'*Sd';

K = [Ad, Bu; Cd, Ddd]*[sdpVars.X; sdpVars.Upsilon];
K1 = K(1:nc, :);
K2 = K(nc+1:end, :);

L = [sdpVars.Y, sdpVars.Theta]*[A_bar*Ad, A_bar*Bd; Cy, Dyd];
L1 = L(:, 1:nx);
L2 = L(:, nx+1:end);

if all(eig(Rd)==0)

    %Diagonal entries
    entry11 = sdpVars.Y;
    entry22 = sdpVars.X;
    entry33 = Z;
    entry44 = eye(rc);
    entry55 = sdpVars.Y;
    entry66 = sdpVars.X;
    entry77 = eye(rb);

    %Lower triangular entries
    entry21 = eye(nc, nx);
    entry31 = Sd*Omega_c;
    entry32 = Sd*K2;
    entry41 = C_bar*Omega_a;
    entry42 = C_bar*K1;
    entry43 = C_bar*Omega_b;
    entry51 = L1;
    entry52 = sdpVars.Gamma;
    entry53 = L2;
    entry54 = zeros(nx, rc);
    entry61 = A_bar*Omega_a;
    entry62 = A_bar*K1;
    entry63 = A_bar*Omega_b;
    entry64 = zeros(nc, rc);
    entry65 = eye(nc, nx);
    entry71 = zeros(rb, nx);
    entry72 = zeros(rb, nc);
    entry73 = zeros(rb, nwd);
    entry74 = zeros(rb, rc);
    entry75 = B_bar'*sdpVars.Y;
    entry76 = B_bar';

    dissSynthesisMatrix = [entry11, entry21', entry31', entry41', entry51', entry61', entry71';...
        entry21, entry22, entry32', entry42', entry52', entry62', entry72';...
        entry31, entry32, entry33, entry43', entry53', entry63', entry73';...
        entry41, entry42, entry43, entry44, entry54', entry64', entry74';...
        entry51, entry52, entry53, entry54, entry55, entry65', entry75';...
        entry61, entry62, entry63, entry64, entry65, entry66, entry76';...
        entry71, entry72, entry73, entry74, entry75, entry76, entry77];
else

    %Diagonal entries
    entry11 = sdpVars.Y;
    entry22 = sdpVars.X;
    entry33 = Z;
    entry44 = -inv(Rd);
    entry55 = eye(rc);
    entry66 = sdpVars.Y;
    entry77 = sdpVars.X;
    entry88 = eye(rb);

    %Lower triangular entries
    entry21 = eye(nc, nx);
    entry31 = Sd*Omega_c;
    entry32 = Sd*K2;
    entry41 = Omega_c;
    entry42 = K2;
    entry43 = Omega_d;
    entry51 = C_bar*Omega_a;
    entry52 = C_bar*K1;
    entry53 = C_bar*Omega_b;
    entry54 = zeros(rc, nzd);
    entry61 = L1;
    entry62 = sdpVars.Gamma;
    entry63 = L2;
    entry64 = zeros(nx, nzd);
    entry65 = zeros(nx, rc);
    entry71 = A_bar*Omega_a;
    entry72 = A_bar*K1;
    entry73 = A_bar*Omega_b;
    entry74 = zeros(nc, nzd);
    entry75 = zeros(nc, rc);
    entry76 = eye(nc, nx);
    entry81 = zeros(rb, nx);
    entry82 = zeros(rb, nc);
    entry83 = zeros(rb, nwd);
    entry84 = zeros(rb, nzd);
    entry85 = zeros(rb, rc);
    entry86 = B_bar'*sdpVars.Y;
    entry87 = B_bar';

    dissSynthesisMatrix = [entry11, entry21', entry31', entry41', entry51', entry61', entry71', entry81';...
        entry21, entry22, entry32', entry42', entry52', entry62', entry72', entry82';...
        entry31, entry32, entry33, entry43', entry53', entry63', entry73', entry83';...
        entry41, entry42, entry43, entry44, entry54', entry64', entry74', entry84';...
        entry51, entry52, entry53, entry54, entry55, entry65', entry75', entry85';...
        entry61, entry62, entry63, entry64, entry65, entry66, entry76', entry86';...
        entry71, entry72, entry73, entry74, entry75, entry76, entry77, entry87';...
        entry81, entry82, entry83, entry84, entry85, entry86, entry87, entry88];
end

numAcc = opts.LMI.numericalAccuracy;
LMI = dissSynthesisMatrix >= numAcc*eye(size(jfAnalysisMatrix));

end