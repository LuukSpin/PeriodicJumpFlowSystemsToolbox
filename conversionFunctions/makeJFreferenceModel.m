function TJF_ref = makeJFreferenceModel(Tct, n_zc, n_wc)

Tct = minreal(ss(Tct));

A = Tct.A;
Bc = Tct.B(:, 1:n_wc);
Bd = Tct.B(:, n_wc+1:end);
Cc = Tct.C(1:n_zc, :);
Cd = Tct.C(n_zc+1:end, :);

Dcc = Tct.D(1:n_zc, 1:n_wc);
Dcd = Tct.D(1:n_zc, n_wc+1:end);
Ddc = Tct.D(n_zc+1:end, 1:n_wc);
Ddd = Tct.D(n_zc+1:end, n_wc+1:end);

nx = size(A, 1);
n_wd = size(Bd, 2);
n_zd = size(Cd, 1);

TJF_ref = JumpFlowSystem();

%Flow matrices
TJF_ref.Ac = [A, Bd; zeros(n_wd, nx), zeros(n_wd, n_wc)];
TJF_ref.Bwc = [Bc; zeros(n_wd, n_wc)];

%Jump matrices
TJF_ref.Ad = blkdiag(eye(nx), zeros(n_wd));
TJF_ref.Bwd = [zeros(nx, n_wd); eye(n_wd)];

%Continuous-time performance channels
TJF_ref.Czc = [Cc Dcd];
TJF_ref.Dzc_wc = Dcc;

%Discrete-time performance channels
TJF_ref.Czd = [Cd Ddd];

checkZeroDdc = all(Ddc==0, 'all');
if ~checkZeroDdc
    error('The matrix Ddc should contain only zeros. Because of the ZOH it is not possible that the disturbance w_d[k] is visible at z_d[k] on the sampling instance!');
end

TJF_ref.Dzd_wd = zeros(n_zd, n_wd); %This is zero because: see error message above

end