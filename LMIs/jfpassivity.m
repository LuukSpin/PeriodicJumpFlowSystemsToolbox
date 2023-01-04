function flag = jfpassivity(objCLJF, opts)
arguments
    objCLJF (1,1) JumpFlowSystem
    opts    (1,1) SDopts
end
% Dimensions;
nx = objCLJF.nx;

% LMI variables
Ph = sdpvar(nx, nx, 'symmetric');

% Stability check
stabCheck = objCLJF.isstable(opts);

if stabCheck
% Fill the H-infinity LMI
HinfLMIAnalysisMatrix = fillPassiveAnalysisLMI(objCLJF, Ph, opts);
HinfAnalysisLMI = (HinfLMIAnalysisMatrix+HinfLMIAnalysisMatrix')/2 >= 1e-9*eye(size(HinfLMIAnalysisMatrix));
else
    warning('The closed-loop jump-flow system is not stable, and hence not passive');
    return
end

% Add a constraint
constraint = Ph >= 1e-9*eye(size(Ph));
%             constraint2 = [Ph, Ph*B_hat; B_hat'*Ph, eye(size(B_hat, 2))];
%             constraint2 = constraint2 >= 1e-9*eye(size(constraint2));

%     diagnostics = optimize(HinfLMI+constraint, [], opts.LMI)
rng(1);
%             diagnostics = optimize(HinfAnalysisLMI+constraint+constraint2, [], opts.LMI);
diagnostics = optimize(HinfAnalysisLMI+constraint, [], opts.LMI.solverOptions);

if diagnostics.problem==0
    flag = true;
else
    flag = false;
end

end