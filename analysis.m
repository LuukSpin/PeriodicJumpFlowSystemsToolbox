function normValue = analysis(sys, opts)

arguments
    sys              {mustBeDynamicalSystem(sys)}
    opts             jfopt
end

if isa(sys, 'InputOutputModel')
    if sys.Ts ~= 0
        sys = dt2jf(sys);
    else
        sys = ct2jf(sys);
    end
elseif isa(sys, 'OpenLoopSampledDataSystem')
    sys = sd2jf(sys);
elseif isa(sys, 'OpenLoopJumpFlowSystem')
    sys = jf2jf(sys);
end

% Check stability
if ~sys.isstable(opts)
    warning('The system is not stable and hence does not have a finite norm of any kind.');
    normValue = nan;
    return
end

% Check all specified system norms such as Hinf, H2, H2g, L1
switch lower(opts.performanceString)
    case {'hinf', 'l2', 'h-inf'}
        normValue = bisectionAnalysisLMI(sys, opts);
    case {'h2'}
        warning('The H2 norm has yet to be implemented in the sampled-data toolbox');
        normValue = nan;
    case {'h2g'}
        warning('The generalized H2 norm has yet to be implemented in the sampled-data toolbox');
        normValue = nan;
    case {'l1'}
        warning('The L1 norm has yet to be implemented in the sampled-data toolbox');
        normValue = nan;
    case {'passivity', 'passive', 'pass'}
        if sys.nwc ~= sys.nzc
            error('The dimensions of the CT disturbance channels and CT performance channels must be the same');
        elseif sys.nwd ~= sys.nzd
            error('The dimensions of the DT disturbance channels and DT performance channels must be the same');
        end
        passivityLMI = fillAnalysisLMI(sys, opts);
        diagnostics = optimize(passivityLMI, [], opts.LMI.solverOptions);
        if diagnostics.problem == 0
            normValue = true;
        else
            normValue = false;
            warningChar = ['The system is not passive, the LMI solver returns ', diagnostics.info];
            warning(warningChar);
        end
    case {'qrs', 'quad', 'quadratic'}
        dissipativityLMI = fillAnalysisLMI(sys, opts);
        diagnostics = optimize(dissipativityLMI, [], opts.LMI.solverOptions);
        if diagnostics.problem == 0
            normValue = true;
        else
            normValue = false;
            warningChar = ['The system is not dissipative with respect to the given quadratic performance specifications, the LMI solver returns ', diagnostics.info];
            warning(warningChar);
        end
    otherwise
        error('The chosen performance indicator string is not a valid choice as it does not represent a system norm or gain.');
end
end