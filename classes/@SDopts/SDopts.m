classdef SDopts < handle

    properties
        figure
        bode
        nyquist
        LMI
        simulation
        reconstructor
    end

    methods
        % Constructor
        function obj = SDopts(h)
            arguments
                h (1,1) double {mustBeGreaterThanOrEqual(h, 0)}
            end

            obj.figure.Position.LeftHalf       = [1.8      41.8    766.4   740.8];
            obj.figure.Position.RightHalf      = [769.8    41.8    766.4   740.8];
            obj.figure.Position.FullScreen     = [1        41      1536    748.8];

            obj.bode = plotopts.BodePlotOptions;
            obj.bode.FreqUnits = 'Hz';
            obj.bode.Grid = 'on';
            obj.bode.PhaseWrapping = 'on';
            obj.bode.Title.String = '';
            obj.bode.Title.FontSize = 18;
            obj.bode.Title.Interpreter = 'latex';
            obj.bode.XLabel.FontSize = 18;
            obj.bode.XLabel.Interpreter = 'latex';
            obj.bode.YLabel.FontSize = 18;
            obj.bode.YLabel.Interpreter = 'latex';
            obj.bode.InputLabels.FontSize = 18;
            obj.bode.InputLabels.Interpreter = 'latex';
            obj.bode.OutputLabels.FontSize = 18;
            obj.bode.OutputLabels.Interpreter = 'latex';

            obj.nyquist = plotopts.NyquistPlotOptions;
            obj.nyquist.FreqUnits = 'Hz';
            obj.nyquist.ShowFullContour = 'off';
            obj.nyquist.Title.String = '';
            obj.nyquist.Title.FontSize = 18;
            obj.nyquist.Title.Interpreter = 'latex';
            obj.nyquist.XLabel.FontSize = 18;
            obj.nyquist.XLabel.Interpreter = 'latex';
            obj.nyquist.YLabel.FontSize = 18;
            obj.nyquist.YLabel.Interpreter = 'latex';
            obj.nyquist.InputLabels.FontSize = 18;
            obj.nyquist.InputLabels.Interpreter = 'latex';
            obj.nyquist.OutputLabels.FontSize = 18;
            obj.nyquist.OutputLabels.Interpreter = 'latex';

            obj.LMI.solverOptions = sdpsettings('verbose', 0, 'solver', 'mosek');
            obj.LMI.schurController = 'no';
            obj.LMI.numericalAccuracy = 1e-8;
            obj.LMI.backoffFactor = 1.01;
            obj.LMI.controllerConditioning = 'no';

            obj.simulation.rule = 1;
            obj.simulation.RelTol = 1e-6;
            obj.simulation.MaxStep = h;
            obj.simulation.options = odeset('RelTol', obj.simulation.RelTol, 'MaxStep', obj.simulation.MaxStep);
            obj.simulation.Tend = 300*h;
            obj.simulation.SampleTime = h;

            obj.reconstructor = 'ZOH';
        end
    end
end
