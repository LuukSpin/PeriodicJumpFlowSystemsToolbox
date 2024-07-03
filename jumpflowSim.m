function [t, zc,k, zd] = jumpflowSim(objJF, wc, wd, opts)
%wc and wd should be function handles, otherwise the HyEQsolver cannot
%produce an output.

arguments
    objJF   (1,1) JumpFlowSystem
    wc
    wd
    opts    %{SDopts, jfopts}
end

h = opts.simulation.SampleTime;

% Rule for jumps
% rule = 1 -> priority for jumps
% rule = 2 -> priority for flows
TSPAN = [0 opts.simulation.Tend]; % simulation time interval
JSPAN = [0 ceil(TSPAN(2)/h)]; % simulation interval for discrete jumps

% Flow matrices
Ac = objJF.Ac;
Bwc = objJF.Bwc;

% Jump matrices
Ad = objJF.Ad;
Bwd = objJF.Bwd;

x0 = zeros(size(Ac, 1)+1, 1);
[t, ~, x] = HyEQsolver(@(x,t) flow(x,wc(t), Ac, Bwc), @(x,t) jump(x,Ad,Bwd,wd(t)), @(x) flowset(x,h), @(x) jumpset(x,h), x0, TSPAN, JSPAN, opts.simulation.rule, opts.simulation.options);
ind = find(abs(x(:, end) - h) < 1e-9);
xi = x(:, 1:end-1);
k = t(ind);

% Construct signals
w_c = nan(size(wc(0), 1), length(t));
w_d = nan(size(wd(0), 1), length(t));
for i = 1:length(t)
    w_c(:, i) = wc(t(i));
    w_d(:, i) = wd(t(i));
end

% Continuous-time performance channels
Cc = objJF.Czc;
Dc = objJF.Dzc_wc;
zc = Cc*xi'+Dc*w_c;

% Discrete-time performance channels
Cd = objJF.Czd;
Dd = objJF.Dzd_wd;
zd = Cd*xi(ind, :)'+Dd*w_d(:, ind);

end