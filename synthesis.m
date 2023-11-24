function [K, synthesisNormValue, T] = synthesis(Plant, opts)

arguments
    Plant     {mustBeA(Plant, ["OpenLoopSampledDataSystem", "OpenLoopJumpFlowSystem"])}
    opts      jfopt
end

% Check if the jump-flow system is a generalized plant (check
% if a stabilizing controller exists)
%             if ~OLJFSystem.isgenplant()
%                 error('This system is not a generalized plant and hence cannot be stabilized');
%             end

if strcmpi(Plant.reconstructor, 'unspecified')
    Plant = applyReconstructor(Plant, opts);
end

% Dimensions
nx = Plant.nx;
nc = nx;
nu = Plant.nu;
ny = Plant.ny;

% LMI variables
Y = sdpvar(nx, nx, 'symmetric');
X = sdpvar(nc, nc, 'symmetric');
Gamma = sdpvar(nx, nc, 'full');
Theta = sdpvar(nx, ny, 'full');
Upsilon = sdpvar(nu, nc, 'full');
Omega = sdpvar(nu, ny, 'full');

sdpVars.Y = Y;
sdpVars.X = X;
sdpVars.Gamma = Gamma;
sdpVars.Theta = Theta;
sdpVars.Upsilon = Upsilon;
sdpVars.Omega = Omega;

% Check all specified system norms such as Hinf, H2, H2g, L1
switch lower(opts.performanceString)
    case {'hinf', 'l2', 'h00', 'hoo'}
        %         [K, synthesisNormValue, T] = SDHinfsyn(Plant, opts);
        [K, synthesisNormValue, T] = bisectionDissSynthesisLMI(Plant, sdpVars, opts);
    case {'h2'}
        warning('The H2 norm has yet to be implemented in the jump-flow systems toolbox');
        K = 0;
        synthesisNormValue = nan;
        T = JumpFlowSystem();
    case {'h2g', 'genh2'}
        warning('The generalized H2 norm has yet to be implemented in the jump-flow systems toolbox');
        K = 0;
        synthesisNormValue = nan;
        T = JumpFlowSystem();
    case {'l1'}
        warning('The L1 norm has yet to be implemented in the jump-flow systems toolbox');
        K = 0;
        synthesisNormValue = nan;
        T = JumpFlowSystem();
    case {'passivity', 'passive', 'pass'}
        LMI = fillDissSynthesisLMI(Plant, sdpVars, opts);
        diagnostics = optimize(LMI, [], opts.LMI.solverOptions);
        if strcmpi(opts.LMI.controllerConditioning, 'yes')
            K = controllerConditioning(Plant, sdpVars, opts);
            T = lft(Plant, K, opts);
        else
            K = controllerConstruction(Plant, sdpVars, opts);
            if ~wellConditionedController(K)
                warning('Controller is ill-conditioned, adding numerical conditioning is advised.');
            end
            T = lft(Plant, K, opts);
            if isstable(T, opts) ~= true
                warning('Closed-loop system is unstable, possibly due to ill-conditioned controller');
            end
        end
        synthesisNormValue = analysis(T, opts);
    case {'qrs', 'quad', 'quadratic'}
        [K, synthesisNormValue, T] = fillDissSynthesisLMI(Plant, sdpVars, opts);
    otherwise
        error('The chosen performance indicator string is not a valid choice as it does not represent a system norm or gain.');
end
end