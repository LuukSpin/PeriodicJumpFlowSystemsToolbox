function [t, zc,k, zd] = jumpflowSim(objFJ, wc, wd, h, opts)
%wc and wd should be function handles, otherwise the HyEQsolver cannot
%produce an output.

arguments
    objFJ   (1,1) JumpFlowSystem
    wc
    wd
    h       (1,1) double
    opts    SDopts = SDopts(h)
end


% Rule for jumps
% rule = 1 -> priority for jumps
% rule = 2 -> priority for flows
TSPAN = [0 opts.simulation.Tend]; % simulation time interval
JSPAN = [0 ceil(TSPAN(2)/h)]; % simulation interval for discrete jumps

% Flow matrices
Ac = objFJ.Ac;
Bc = objFJ.Bwc;

% Jump matrices
Ad = objFJ.Ad;
Bd = objFJ.Bwd;

x0 = zeros(size(Ac, 1)+1, 1);
[t, ~, x] = HyEQsolver(@(x,t) f(x,wc(t), Ac, Bc), @(x,t) g(x,Ad,Bd,wd(t)), @(x) flowset(x,h), @(x) jumpset(x,h), x0, TSPAN, JSPAN, obj.simulation.rule, obj.simulation.options);
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
Cc = objFJ.Czc;
Dc = objFJ.Dzc_wc;
zc = Cc*xi'+Dc*w_c;

% Discrete-time performance channels
Cd = objFJ.Czd;
Dd = objFJ.Dzd_wd;
zd = Cd*xi(ind, :)'+Dd*w_d(:, ind);

end