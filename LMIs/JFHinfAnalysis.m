function gamma = JFHinfAnalysis(objCLJF, h)
% Dimensions;
nx = size(objCLJF.Ac, 1);

% LMI variables
Ph = sdpvar(nx, nx, 'symmetric');

% Bisection settings
Nmax = 100; %Maximum numbers of iterations
tol = 1e-2; %Tolerance to calculate upperbound of Hinf norm

% Initialization
a = [tol 2];
N = 0;
last = 0;
initialfeas = 0;

% Run the bisection based search until gamma is within the specified
% tolerance or the maximum amount of iterations is reached
while N < Nmax
    N = N + 1;
    
    if ~xor((a(2) - a(1)) > tol, N < Nmax-1) || last
        gamma = mean(a);
        
        % Fill the H-infinity LMI
        HinfLMIAnalysisMatrix = fillHinfAnalysisLMI(objCLJF, Ph, h, gamma);
        HinfAnalysisLMI = (HinfLMIAnalysisMatrix+HinfLMIAnalysisMatrix')/2 >= 1e-9*eye(size(HinfLMIAnalysisMatrix));
        
        
        % Add a constraint
        constraint = Ph >= 1e-9;
        
        opts = LS.opts;
        %     diagnostics = optimize(HinfLMI+constraint, [], opts.LMI)
        rng(1);
        diagnostics = optimize(HinfAnalysisLMI+constraint, [], opts.LMI);
        
        if (value(HinfAnalysisLMI) && value(constraint))
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
    if JFStability(objCLJF, h)
        gamma = a(2);
    else
        gamma = nan;
        return
    end
else
    gamma = nan;
    return
end

end