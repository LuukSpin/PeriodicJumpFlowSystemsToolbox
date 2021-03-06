function [A_hat, B_hat, C_hat] = HamiltonianSD(Ac, Bc, Cc, Dc, gamma, opts)
%HAMILTONIANJF determines the 
%   Detailed explanation goes here

h = opts.simulation.SampleTime;
n = size(Ac, 1);

Mc_inv = eye(size(Dc'*Dc))*gamma^2-Dc'*Dc;
Lc_inv = eye(size(Dc*Dc'))*gamma^2-Dc*Dc';

% Fill Hamiltonian
H = [Ac+Bc/Mc_inv*(Dc')*Cc, gamma*Bc/Mc_inv*(Bc'); -gamma*(Cc')/Lc_inv*Cc, -(Ac+Bc/Mc_inv*(Dc')*Cc)'];

% Calculate exponential of the Hamiltonian
F = expm(-H*h);
F11 = F(1:n, 1:n);
F12 = F(1:n, n+1:end);
F21 = F(n+1:end, 1:n);

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
