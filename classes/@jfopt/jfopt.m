classdef jfopt < handle

    properties
        simulation
        LMI
        reconstructor
        performanceString
        performanceValue
        numericalConditioning
    end

    methods
        % Constructor
        function obj = jfopt(h, performanceString, performanceValue, numericalConditioning)
            arguments
                h                       (1,1) double {mustBeGreaterThanOrEqual(h, 0)}
                performanceString       char = 'hinf'
                performanceValue        double = []
                numericalConditioning   char = 'no'
            end

            obj.simulation.rule = 1;
            obj.simulation.RelTol = 1e-6;
            obj.simulation.MaxStep = h;
            obj.simulation.SampleTime = h;
            obj.simulation.options = odeset('RelTol', obj.simulation.RelTol, 'MaxStep', obj.simulation.MaxStep);
            obj.simulation.Tend = 300*h;

            obj.LMI.solverOptions = sdpsettings('verbose', 0, 'solver', 'mosek');
            obj.LMI.numericalAccuracy = 1e-8;
            obj.LMI.bisection.maxIter = 100;
            obj.LMI.bisection.tol = 1e-4;
            obj.LMI.backoffFactor = 1.01;
            obj.LMI.controllerConditioning = 'yes';
            obj.LMI.numericalConditioning = numericalConditioning;

            obj.performanceString = performanceString;

            obj.performanceValue = performanceValue;

            obj.reconstructor = 'ZOH';
        end
    end
end
