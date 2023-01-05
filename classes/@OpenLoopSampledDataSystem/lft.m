function objJF = lft(OLSDSystem1, OLSDSystem2, opts)
%LFT is used to interconnect systems
%   LFT(sys1, sys2) produces a closed-loop jump-flow system.
%   sys1 has to be an OpenLoopJumpFlowSystem while sys2 may be
%   a discrete controller. The interconnection looks as
%   follows:
%

arguments
    OLSDSystem1         OpenLoopSampledDataSystem
    OLSDSystem2         {mustBeNumericOrListedType(OLSDSystem2, "ss", "tf", "OpenLoopSampledDataSystem")}
    opts                jfopt
end

if strcmpi(OLSDSystem1.reconstructor, 'unspecified')
    OLSDSystem1 = applyReconstructor(OLSDSystem1, opts);
end

% If the second input is a controller than convert it to a
% OLSDSystem
if ~isa(OLSDSystem2, 'OpenLoopSampledDataSystem')
    OLSDSystem2 = ss(OLSDSystem2);
    [p, m] = size( OLSDSystem2);
    OLSDSystem2 = dt2sd(OLSDSystem2, p, m);
else
    if strcmpi(OLSDSystem2.reconstructor, 'unspecified')
        OLSDSystem2 = applyReconstructor(OLSDSystem2, opts);
    end
end

% Make sure that the interconnection has a compatible
% connection
y1 = OLSDSystem1.ny;
u1 = OLSDSystem1.nu;
y2 = OLSDSystem2.ny;
u2 = OLSDSystem2.nu;

if (y1 ~= u2) || (u1 ~= y2)
    error('The interconnection is not possible because the dimensions of the connection does not match');
end

Dyu1 = OLSDSystem1.Dy_ud;
Dyu2 = OLSDSystem2.Dy_ud;

% Interconnection must be wellposed, can be checked by
% feed-through terms
wellPosednessMatrix = [eye(y2), -Dyu2; -Dyu1, eye(y1)];
if det(wellPosednessMatrix) == 0
    error('This interconnection is not well-posed, and hence the interconnection will result in a non-causal system. The interconnection is aborted.');
end

% Flow and continuous-time performance channels matrices
Ac = blkdiag(OLSDSystem1.Ac, OLSDSystem2.Ac);
Bwc = blkdiag(OLSDSystem1.Bwc, OLSDSystem2.Bwc);
Czc = blkdiag(OLSDSystem1.Czc, OLSDSystem2.Czc);
Dzc_wc= blkdiag(OLSDSystem1.Dzc_wc, OLSDSystem2.Dzc_wc);

% Define matrices used in the partitioning that follows
R12 = eye(y1)-Dyu1*Dyu2;
R21 = eye(y2)-Dyu2*Dyu1;
Ad1 = OLSDSystem1.Ad;
Ad2 = OLSDSystem2.Ad;
Bu1 = OLSDSystem1.Bud;
Bu2 = OLSDSystem2.Bud;
Cy1 = OLSDSystem1.Cy;
Cy2 = OLSDSystem2.Cy;
Bd1 = OLSDSystem1.Bwd;
Bd2 = OLSDSystem2.Bwd;
Dyd1 = OLSDSystem1.Dy_wd;
Dyd2 = OLSDSystem2.Dy_wd;
Cd1 = OLSDSystem1.Czd;
Cd2 = OLSDSystem2.Czd;
Ddu1 = OLSDSystem1.Dzd_ud;
Ddu2 = OLSDSystem2.Dzd_ud;
Ddd1 = OLSDSystem1.Dzd_wd;
Ddd2 = OLSDSystem2.Dzd_wd;

% Jump and discrete-time performance channels matrices
Ad = [Ad1 + Bu1/R21*Dyu2*Cy1, Bu1/R21*Cy2; Bu2/R12*Cy1, Ad2 + Bu2/R12*Dyu1*Cy2];
Bwd = [Bd1 + Bu1/R21*Dyu2*Dyd1, Bu1/R21*Dyd2; Bu2/R12*Dyd1, Bd2 + Bu2/R12*Dyu1*Dyd2];
Czd = [Cd1 + Ddu1/R21*Dyu2*Cy1, Ddu1/R21*Cy2; Ddu2/R12*Cy1, Cd2 + Ddu2/R12*Dyu1*Cy2];
Dzd_wd = [Ddd1 + Ddu1/R21*Dyu2*Dyd1, Ddu1/R21*Dyd2; Ddu2/R12*Dyd1, Ddd2 + Ddu2/R12*Dyu1*Dyd2];

objJF = JumpFlowSystem(Ac, Bwc, Ad, Bwd, Czc, Dzc_wc, Czd, Dzd_wd);

% Check stability if second input is a controller
if ~isa(OLSDSystem2, 'OpenLoopSampledDataSystem')
    if ~objJF.isstable(opts)
        warning('The closed-loop interconnection of the two systems is not stable!');
    end
end
end