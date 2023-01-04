clear variables
clc
close all
m = 4;
p = 4;

rng shuffle
flag = true;

while flag
    P_dt = drss(10, p, m);
    if ~(max(abs(tzero(P_dt))) >= 1)
        flag = false;
    end
end

P_dt = minreal(P_dt);

nmeas = p-3; %ny
ncont = m-3; %nu

A = P_dt.A;

nx = size(A, 1);
nwc = 0;
nzc = 0;
nwd = m-ncont;
nzd = p-nmeas;
nu = ncont;
ny = nmeas;

Bw = P_dt.B(:, 1:nwd);
Bu = P_dt.B(:, nwd+1:end);
Cz = P_dt.C(1:nzd, :);
Cy = P_dt.C(nzd+1:end, :);
Dzw = P_dt.D(1:nzd,1:nwd);
Dzu = P_dt.D(1:nzd, (nwd+1):end);
Dyw = P_dt.D((nzd+1):end, 1:nwd);
Dyu = zeros(nmeas, ncont);

h = 1e-3;

P_dt = minreal(ss(A, [Bw, Bu], [Cz; Cy], [Dzw, Dzu; Dyw, Dyu], h));

[K_dt, ~, gamma_dt] = hinfsyn(P_dt, nmeas, ncont);
gamma_dt
K_dt = minreal(K_dt);
T_dt = lft(P_dt, K_dt);
gamma_dtdt = hinfnorm(T_dt)


P_sd = OpenLoopSampledDataSystem(zeros(nx), zeros(nx, nwc), zeros(nx, nu), A, Bw, Bu, zeros(nzc, nx), zeros(nzc, nwc), zeros(nzc, nu), Cz, Dzw, Dzu, Cy, Dyw, Dyu);
opts = SDopts(h);
[K_sd, gamma_sd, ~] = synthesis(P_sd, 'hinf', opts);
gamma_sd
K_sd = minreal(K_sd);
T_sd = lft(P_sd, K_sd, opts);
gamma_sdsd = analysis(T_sd, 'hinf', opts)

gamma_dtsd = hinfnorm(lft(P_dt, K_sd))
gamma_sddt = analysis(lft(P_sd, K_dt, opts), 'hinf', opts)

bode(K_dt, K_sd);
figure
pzmap(K_dt, K_sd);








