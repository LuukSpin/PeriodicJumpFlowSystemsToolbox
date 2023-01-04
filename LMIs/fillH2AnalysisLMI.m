function [outputArg1,outputArg2] = fillH2LMI(inputArg1,inputArg2)



gamma = sdpvar(1, 1, 'full');
diagnostics = optimize(LMI, gamma, opts.LMI);


end

