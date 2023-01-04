function [Controller, synthesisNormValue, CLJFSystem] = synthesis(Plant, opts)

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

% Check all specified system norms such as Hinf, H2, H2g, L1
switch lower(opts.performanceString)
    case {'hinf', 'l2', 'h00', 'hoo'}
        [Controller, synthesisNormValue, CLJFSystem] = SDHinfsyn(Plant, opts);
    case {'h2'}
        warning('The H2 norm has yet to be implemented in the jump-flow systems toolbox');
        Controller = 0;
        synthesisNormValue = nan;
        CLJFSystem = JumpFlowSystem();
    case {'h2g', 'genh2'}
        warning('The generalized H2 norm has yet to be implemented in the jump-flow systems toolbox');
        Controller = 0;
        synthesisNormValue = nan;
        CLJFSystem = JumpFlowSystem();
    case {'l1'}
        warning('The L1 norm has yet to be implemented in the jump-flow systems toolbox');
        Controller = 0;
        synthesisNormValue = nan;
        CLJFSystem = JumpFlowSystem();
    case {'passivity', 'passive', 'pass'}
        [Controller, synthesisNormValue, CLJFSystem] = SDPassivesyn(Plant, opts);
    case {'qrs', 'quad', 'quadratic'}
        warning('Controller synthesis based on quadratic dissipativity is not yet implemented in the jump-flow systems toolbox');
        Controller = 0;
        synthesisNormValue = nan;
        CLJFSystem = JumpFlowSystem();
    otherwise
        error('The chosen performance indicator string is not a valid choice as it does not represent a system norm or gain.');
end
end