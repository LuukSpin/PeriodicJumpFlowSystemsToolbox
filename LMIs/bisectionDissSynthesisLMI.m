function synVars = bisectionDissSynthesisLMI(sys, opts)
arguments
    sys     OpenLoopSampledDataSystem
    opts    jfopt
end

% Bisection settings
Nmax = opts.LMI.bisection.maxIter; %Maximum numbers of iterations of the bisection search
tol = opts.LMI.bisection.tol; %Tolerance to calculate upperbound of Hinf norm
numAcc = opts.LMI.numericalAccuracy;

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



if strcmpi(opts.numericalConditioning, 'on')
    
end

end



