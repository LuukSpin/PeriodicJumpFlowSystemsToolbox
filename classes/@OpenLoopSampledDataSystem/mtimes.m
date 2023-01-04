function SDSystem = mtimes(obj1, obj2)
% MTIMES specifies the multiplication operator * for sampled-data
% systems
%   Multiplication of sampled-data systems specifies a series
%   connection of system that looks like the following block
%   diagram:
%
%

nx1 = obj2.nx;
nx2 = obj1.nx;
nzc1 = obj2.nzc;
nzd1 = obj2.nzd;
ny1 = obj2.ny;
nwc2 = obj1.nwc;
nwd2 = obj1.nwd;
nu2 = obj1.nu;

if nzc1 ~= nwc2
    error('The amount of continuous-time performance channels of the first system is not equal to the amount of continuous-time disturbance channels of the second system.');
end

if nzd1 ~= nwd2
    error('The amount of discrete-time performance channels of the first system is not equal to the amount of discrete-time disturbance channels of the second system.');
end

if ny1 ~= nu2
    error('The controller input dimension of the first system does not match the controller output dimension of the second system.');
end

Ac = [obj2.Ac, zeros(nx1, nx2); obj1.Bwc*obj2.Czc, obj1.Ac];
Bwc = [obj2.Bwc; obj1.Bwc*obj2.Dzc_wc];
%             Buc =

Ad = [obj2.Ad, zeros(nx1, nx2); obj1.Bwd*obj2.Czd+obj1.Bud*obj2.Cy, obj1.Ad];
Bwd = [obj2.Bwd; obj1.Bwd*obj2.Dzd_wd+obj1.Bud*obj2.Dy_wd];
Bud = [obj2.Bud; obj1.Bwd*obj2.Dzd_u+obj1.Bud*obj2.Dy_u];

Czc = [obj1.Dzc_wc*obj2.Czc, obj1.Czc];
Dzc_wc = obj1.Dzc_wc*obj2.Dzc_wc;
%             Dzc_uc =

Czd = [obj1.Dzd_wd*obj2.Czd+obj1.Dzd_uc*obj2.Cy, obj1.Czd];
Dzd_wd = obj1.Dzd_wd*obj2.Dzd_wd+obj1.Dzd_uc*obj2.Dy_wd;
Dzd_u = obj1.Dzd_wd*obj2.Dzd_u+obj1.Dzd_uc*obj2.Dy_u;

Cy = [obj1.Dy_wd*obj2.Czd+obj1.Dy_u*obj2.Cy, obj1.Cy];
Dy_wd = obj1.Dy_wd*obj2.Dzd_wd+obj1.Dy_u*obj2.Dy_wd;
Dy_u = obj1.Dy_wd*obj2.Dzd_u+obj1.Dy_u*obj2.Dy_u;

SDSystem = OpenLoopSampledDataSystem(Ac, Bwc, Ad, Bwd, Bud, Czc, Dzc_wc, Czd, Dzd_wd, Dzd_u, Cy, Dy_wd, Dy_u);

endfunction [outputArg1,outputArg2] = untitled6(inputArg1,inputArg2)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
outputArg1 = inputArg1;
outputArg2 = inputArg2;
end