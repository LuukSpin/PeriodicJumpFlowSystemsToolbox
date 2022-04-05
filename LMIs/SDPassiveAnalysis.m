function flag = SDPassiveAnalysis(objCLJF, h, opts)
arguments
    objCLJF (1,1) JumpFlowSystem
    h       (1,1) double
    opts    (1,1) SDopts = SDopts()
end
% Dimensions;
nx = objCLJF.nx;

% LMI variables
Ph = sdpvar(nx, nx, 'symmetric');

% Stability check
stabCheck = objCLJF.isstable(h);

if stabCheck
% Fill the H-infinity LMI
HinfLMIAnalysisMatrix = fillPassiveAnalysisLMI(objCLJF, Ph, h);
HinfAnalysisLMI = (HinfLMIAnalysisMatrix+HinfLMIAnalysisMatrix')/2 >= 1e-9*eye(size(HinfLMIAnalysisMatrix));
else
    warning('The closed-loop jump-flow system is not stable, and hence not passive');
    return
end


% Add a constraint
constraint = Ph >= 1e-9*eye(size(Ph));
%             constraint2 = [Ph, Ph*B_hat; B_hat'*Ph, eye(size(B_hat, 2))];
%             constraint2 = constraint2 >= 1e-9*eye(size(constraint2));

opts = LS.opts;
%     diagnostics = optimize(HinfLMI+constraint, [], opts.LMI)
rng(1);
%             diagnostics = optimize(HinfAnalysisLMI+constraint+constraint2, [], opts.LMI);
diagnostics = optimize(HinfAnalysisLMI+constraint, [], opts.LMI);

if diagnostics.problem==0
    flag = true;
else
    flag = false;
end

end