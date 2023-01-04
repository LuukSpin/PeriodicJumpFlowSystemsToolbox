function [Controller, synthesisNormValue, CLJFSystem] = synthesis(Plant, performanceIndicator, opts)

arguments
    Plant                   (1,1) {mustBeA(Plant, ["JumpFlowSystem", "OpenLoopSampledDataSystem", "OpenLoopJumpFlowSystem"])}
    performanceIndicator    (1,1) char
    opts                    (1,1) SDopts
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
switch performanceIndicator
    case {'Hinf', 'L2', 'H-inf', 'hinf', 'l2', 'h-inf'}
        [Controller, synthesisNormValue, CLJFSystem] = SDHinfsyn(Plant, opts);
    case {'H2', 'h2'}
        warning('The H2 norm has yet to be implemented in the jump-flow systems toolbox');
        Controller = 0;
        synthesisNormValue = nan;
        CLJFSystem = JumpFlowSystem();
    case {'H2g', 'h2g', 'genH2', 'genh2', 'GenH2', 'Genh2'}
        warning('The generalized H2 norm has yet to be implemented in the jump-flow systems toolbox');
        Controller = 0;
        synthesisNormValue = nan;
        CLJFSystem = JumpFlowSystem();
    case {'L1', 'l1'}
        warning('The L1 norm has yet to be implemented in the jump-flow systems toolbox');
        Controller = 0;
        synthesisNormValue = nan;
        CLJFSystem = JumpFlowSystem();
    case {'Passivity', 'passivity', 'Passive', 'passive', 'Pass', 'pass'}
        [Controller, synthesisNormValue, CLJFSystem] = SDPassivesyn(Plant, opts);
    case {'QRS', 'Quad', 'Quadratic', 'qrs', 'quad', 'quadratic'}
        warning('Controller synthesis based on quadratic dissipativity is not yet implemented in the jump-flow systems toolbox');
        Controller = 0;
        synthesisNormValue = nan;
        CLJFSystem = JumpFlowSystem();
    otherwise
        error('The chosen performance indicator string is not a valid choice as it does not represent a system norm or gain.');
end
end