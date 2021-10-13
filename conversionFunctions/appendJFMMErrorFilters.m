function weightedSDSystem = appendJFMMErrorFilters(unweightedSDSystem, Wzc, Wzd)
% APPENDSDWEIGHTINGFILTERS combines weighting filters with an open-loop
% sampled-data system
%
%  This function appends weighted filters to an open-loop Sampled-Data
%  system. More specifically it transforms a JF system with state-space 
%  realization of the form:
%
%  \dot{x} = Ac*x  + Bwc*w_c
%   x^+    = Ad*x  + Bwd*w_d    + Bud*\hat{u}
%   e_c    = Czc*x + Dzc_wc*w_c
%   e_d    = Czd*x + Dzd_wd*w_d + Dzd_u*\hat{u}
%   y      = Cy*x  + Dy_wd*w_d
%
%  Into a JF system with state-space realization of the form:
%
%  \dot{x} = Ac*x  + Bwc*w_c
%   x^+    = Ad*x  + Bwd*w_d    + Bud*\hat{u}
%   e_c    = Czc*x + Dzc_wc*w_c
%   e_d    = Czd*x + Dzd_wd*w_d + Dzd_u*\hat{u}
%   y      = Cy*x  + Dy_wd*w_d
%
%           +--------+      +---------+
%           |        |      |         |
%  w_c ---> |        | ---> | W_{e_c} | ---> e_c
%           |        |      |         |
%           |        |      +---------+
%           |        |      +---------+
%           |  MMJF  |      |         |
%  w_d ---> |        | ---> | W_{e_d} | ---> e_d
%           |        |      |         |
%           |        |      +---------+
%           |        |
%  u -----> |        | --------------------> y
%           +--------+

% Check that filter Wzc is ss (or tf) class and continuous-time
if ~(isa(Wzc, 'ss') || isa(Wzc, 'tf'))
    error('The weighting filter for the continuous-time performance channel should be of class "ss" or "tf"');
else
    Wzc = ss(Wzc);
    if Wzc.Ts ~= 0
        error('The weighting filter for the continuous-time performance channel should be a continuous-time filter');
    end
end

% Check that filter Wzd is ss (or tf) class and discrete-time
if ~(isa(Wzd, 'ss') || isa(Wzd, 'tf'))
    error('The weighting filter for the discrete-time performance channel should be of class "ss" or "tf"');
else
    Wzd = ss(Wzd);
    if Wzd.Ts == 0
        error('The weighting filter for the discrete-time performance channel should be a discrete-time filter');
    end
end

% Check dimensions between the filters and the unweighted SD system
% Check n_zc
if ~(size(unweightedSDSystem.Czc, 1) == size(Wzc, 2))
    error('The dimensions of the continuous-time performance channel of the unweighted SD system and the filter "Wzc" does not match');
end
% Check n_zd
if ~(size(unweightedSDSystem.Czd, 1) == size(Wzd, 2))
    error('The dimensions of the discrete-time performance channel of the unweighted SD system and the filter "Wzc" does not match');
end

%IO-Dimensions
n_wd = size(unweightedSDSystem.Bwd, 2);
n_zd = size(unweightedSDSystem.Czd, 1);
nu = size(unweightedSDSystem.Buc, 2);
ny = size(unweightedSDSystem.Cy, 1);

%State dimensions
nx_sd = size(unweightedSDSystem.Ac, 1);
nx_wc = size(Wwc.A, 1);
nx_zc = size(Wzc.A, 1);
nx_wd = size(Wwd.A, 1);
nx_zd = size(Wzd.A, 1);

% The new state dimensions is x_weight = col(nx_sd, nx_wc, nx_zc, nx_wd, nx_zd)
% Initialize a OL SD class for the weighted SD system
weightedSDSystem = OpenLoopJumpFlowSystem();

%Flow matrices of the weighted SD system
weightedSDSystem.Ac = [unweightedSDSystem.Ac, unweightedSDSystem.Bwc*Wwc.C, zeros(nx_sd, nx_zc), zeros(nx_sd, nx_wd), zeros(nx_sd, nx_zd);...
                       zeros(nx_wc, nx_sd), Wwc.A, zeros(nx_wc, nx_zc), zeros(nx_wc, nx_wd), zeros(nx_wc, nx_zd);...
                       Wzc.B*unweightedSDSystem.Czc, Wzc.B*unweightedSDSystem.Dzc_wc*Wwc.C, Wzc.A, zeros(nx_zc, nx_wd), zeros(nx_zc, nx_zd);...
                       zeros(nx_wd, nx_sd+nx_wc+nx_zc+nx_wd+nx_zd);...
                       zeros(nx_zd, nx_sd+nx_wc+nx_zc+nx_wd+nx_zd)];
weightedSDSystem.Bwc = [unweightedSDSystem.Bwc*Wwc.D; Wwc.B; Wzc.B*unweightedSDSystem.Dzc_wc*Wwc.D; zeros(nx_wd+nx_zd, n_wc)];

%Jump matrices of the weighted SD system
weightedSDSystem.Ad = [eye(nx_sd+nx_wc+nx_zc), zeros(nx_sd+nx_wc+nx_zc, nx_wd+nx_zd);...
                       zeros(nx_wd, nx_sd+nx_wc+nx_zc), Wwd.A, zeros(nx_wd, nx_zd);...
                       Wzd.B*unweightedSDSystem.Czd, zeros(nx_zd, nx_wc+nx_zc), Wzd.B*unweightedSDSystem.Dzd_wd*Wwd.C, Wzd.A];
weightedSDSystem.Bwd = [zeros(nx_sd+nx_wc+nx_zc, n_wd); Wwd.B; Wzd.B*unweightedSDSystem.Dzd_wd*Wwd.D];
weightedSDSystem.Bud = [zeros(nx_sd+nx_wc+nx_zc+nx_wd, nu); Wzd.B*unweightedSDSystem.Dzd_u];

%Continuous-time performance channels of weighted SD system
weightedSDSystem.Czc = [Wzc.D*unweightedSDSystem.Czc, Wzc.D*unweightedSDSystem.Dzc_wc*Wwc.C, Wzc.C, zeros(n_zc, nx_wd+nx_zd)];
weightedSDSystem.Dzc_wc = Wzc.D*unweightedSDSystem.Dzc_wc*Wwc.D;

%Discrete-time performance channels of weighted SD system
weightedSDSystem.Czd = [Wzd.D*unweightedSDSystem.Czd, zeros(n_zd, nx_wc+nx_zc), Wzd.D*unweightedSDSystem.Dzd_wd*Wwd.C, Wzd.C];
weightedSDSystem.Dzd_wd = Wzd.D*unweightedSDSystem.Dzd_wd*Wwd.D;
weightedSDSystem.Dzd_u = Wzd.D*unweightedSDSystem.Dzd_u;

%Output matrices of the weighted SD system
weightedSDSystem.Cy = [unweightedSDSystem.Cy, zeros(ny, nx_wc+nx_zc), unweightedSDSystem.Dy_wd*Wwd.C, zeros(ny, nx_zd)];
weightedSDSystem.Dy_wd = unweightedSDSystem.Dy_wd*Wwd.D;
weightedSDSystem.Dy_u = zeros(ny, nu);

end

