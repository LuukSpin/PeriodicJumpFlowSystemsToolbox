function [A_hat, B_hat, C_hat] = jfhamilexp(jfsys, opts)

arguments
    jfsys       (1,1) JumpFlowSystem
    opts        (1,1) jfopt
end

[H, M2] = jfhamiltonian(jfsys, opts);
if any(real(eig(M2))<=0)
    error('The conditition M2>0 should be satisfied in order for the lyapunov matrix to be a solution to the associated differential riccati equation');
end

F = expm(-H*opts.simulation.SampleTime);

nx = jfsys.nx;

F11 = F(1:nx, 1:nx);
F12 = F(1:nx, nx+1:end);
F21 = F(nx+1:end, 1:nx);

% Determine A, B and C_hat
A_hat = inv(F11);
B_hat = -F11\F12;
C_hat = F21/F11;

% Use singular value decomposition to construct B_hat
[Yb, Sb, ~] = svd(B_hat);
rb = rank(Sb);
if rb == 0
    rb = [];
end
B_hat = Yb*sqrt(Sb(:, 1:rb));

% Use singular value decomposition to construct C_hat
[~, Sc, Yc] = svd(C_hat);
rc = rank(Sc);
if rc == 0
    rc = [];
end
C_hat = sqrt(Sc(1:rc, :))*Yc';

end