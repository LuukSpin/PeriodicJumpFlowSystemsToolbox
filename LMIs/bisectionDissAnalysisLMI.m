function gamma = bisectionDissAnalysisLMI(jfsys, opts)
arguments
    jfsys   JumpFlowSystem
    opts    jfopt
end

% Bisection settings
Nmax = opts.LMI.bisection.maxIter; %Maximum numbers of iterations of the bisection search
tol = opts.LMI.bisection.tol; %Tolerance to calculate upperbound of Hinf norm

% Initialization
a_init = max(norm(jfsys.Dzc_wc, 2), norm(jfsys.Dzd_wd, 2));
if a_init == 0
    a = [tol 2];
else
    a = [a_init max(2, 2*a_init)];
end

N = 0;
last = 0;
initialfeas = 0;

% Stability check
if ~isstable(jfsys, opts)
    error('System is not stable and hence does not have a finite Hinf norm');
end

% Run the bisection based search until gamma is within the specified
% tolerance or the maximum amount of iterations is reached
while N < Nmax
    N = N + 1;

    if ~xor((a(2) - a(1)) > tol, N < Nmax-1) || last

        opts.performanceValue = mean(a);

        % Fill the H-infinity LMI
        LMI = fillDissAnalysisLMI(jfsys, opts);

        rng shuffle;
        diagnostics = optimize(LMI, [], opts.LMI.solverOptions);

        if diagnostics.problem == 0
            a(2) = mean(a);
            initialfeas = 1;
        else
            if initialfeas
                a(1) = mean(a);
            else
                a(2) = 2*a(2);
            end
        end
        if last
            break;
        end
    else
        last = 1;
        a(1) = a(2);
    end

end

% Determine if bisection-based search was succesful
if initialfeas
    gamma = a(2);
else
    gamma = nan;
    return
end

end