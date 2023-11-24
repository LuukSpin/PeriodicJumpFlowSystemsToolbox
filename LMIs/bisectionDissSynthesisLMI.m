function [K, val, T] = bisectionDissSynthesisLMI(sys, sdpVars, opts)
% This bisection synthesis algorithm now only works for Hinf norm. Gen H2
% norm will be added in the future.

arguments
    sys     OpenLoopSampledDataSystem
    sdpVars struct
    opts    jfopt
end

% Bisection settings
Nmax = opts.LMI.bisection.maxIter; %Maximum numbers of iterations of the bisection search
tol = opts.LMI.bisection.tol; %Tolerance to calculate upperbound of Hinf norm

% Initialization
a_init = max(norm(sys.Dzc_wc, 2), norm(sys.Dzd_wd, 2));
if a_init == 0
    a = [tol 2];
else
    a = [a_init max(2, 2*a_init)];
end

N = 0;
last = 0;
initialfeas = 0;

% Dimensions
nx = sys.nx;
nc = nx;
nu = sys.nu;
ny = sys.ny;

% Run the bisection based search until gamma is within the specified
% tolerance or the maximum amount of iterations is reached
while N < Nmax
    N = N + 1;

    if ~xor((a(2) - a(1)) > tol, N < Nmax-1) || last
        opts.performanceValue = mean(a);

        % Fill the H-infinity LMI
        synLMI = fillDissSynthesisLMI(sys, sdpVars, opts);

        rng shuffle;
        diagnostics = optimize(synLMI, [], opts.LMI.solverOptions);

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
    opts.performanceValue = a(2);
else
    opts.performanceValue = nan;
    return
end

if strcmpi(opts.LMI.controllerConditioning, 'yes')
    [K, val] = controllerConditioning(sys, sdpVars, opts);
    T = lft(sys, K, opts);
else
    K = controllerConstruction(sys, sdpVars, opts);
    if ~wellConditionedController(K)
        warning('Controller is ill-conditioned, adding numerical conditioning is advised.');
    end
    val = opts.performanceValue;
    T = lft(sys, K, opts);
    if isstable(T, opts) ~= true
        warning('Closed-loop system is unstable, possibly due to ill-conditioned controller');
    end
end

end




