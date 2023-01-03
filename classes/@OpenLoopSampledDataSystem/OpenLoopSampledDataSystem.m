classdef OpenLoopSampledDataSystem < JumpFlowSystem
    %OpenLoopSampledDataSystem Construct an instance of this class
    %   OpenLoopSampledDataSystem(Ac, Bwc, Buc, Ad, Bwd, Bud, Czc,
    %   Dzc_wc, Dzc_uc, Czd, Dzd_Wd, Dzd_ud, Cy, Dy_wd, Dy_ud)
    %   construct an open-loop sampled-data system with the
    %   following state-space realization:
    %  \dot{x} = Ac*x  + Bwc*w_c    + Buc*u
    %   x^+    = Ad*x  + Bwd*w_d    + Bud*\hat{u}
    %   z_c    = Czc*x + Dzc_wc*w_c + Dzc_uc*u
    %   z_d    = Czd*x + Dzd_wd*w_d + Dzd_ud*\hat{u}
    %   y      = Cy*x  + Dy_wd*w_d  + Dy_ud*\hat{u}

    properties (SetAccess = private)
        %Controller output to state matrices
        Buc     double {mustBeFinite(Buc)}
        Bud     double {mustBeFinite(Bud)}

        %Controller output to performance channels feed-through terms
        Dzc_uc  double {mustBeFinite(Dzc_uc)}
        Dzd_ud  double {mustBeFinite(Dzd_ud)}

        %Controller input matrices
        Cy      double {mustBeFinite(Cy)}
        Dy_wd   double {mustBeFinite(Dy_wd)}
        Dy_ud   double {mustBeFinite(Dy_ud)}
    end

    properties (Dependent = true, SetAccess = private, GetAccess = 'public')
        nu      (1,1) double
        ny      (1,1) double
    end

    properties (GetAccess = 'public', SetAccess = {?applyReconstructor, ?appendWeightingFilters})
        reconstructor = 'unspecified'
    end

    methods
        %% Constructor
        function obj = OpenLoopSampledDataSystem(Ac, Bwc, Buc, Ad, Bwd, Bud, Czc, Dzc_wc, Dzc_uc, Czd, Dzd_wd, Dzd_ud, Cy, Dy_wd, Dy_ud)

            % Here the arguments is setup such that the input arguments are
            % variable. That means that this constructor accepts 0 up to 15
            % input arguments. However, no more than 15 input arguments is
            % allowed.
            arguments
                Ac = []
                Bwc = zeros(size(Ac, 1), 0)
                Buc = zeros(size(Ac, 1), 0)
                Ad = zeros(size(Ac))
                Bwd = zeros(size(Ac, 1), 0)
                Bud = zeros(size(Ac, 1), size(Buc, 2))
                Czc = zeros(0, size(Ac, 1))
                Dzc_wc = zeros(size(Czc, 1), size(Bwc, 2))
                Dzc_uc = zeros(size(Czc, 1), size(Buc, 2))
                Czd = zeros(0, size(Ac, 1))
                Dzd_wd = zeros(size(Czd, 1), size(Bwd, 2))
                Dzd_ud = zeros(size(Czd, 1), size(Bud, 2))
                Cy = zeros(0, size(Ac, 1))
                Dy_wd = zeros(size(Cy, 1), size(Bwd, 2))
                Dy_ud = zeros(size(Cy, 1), size(Bud, 2))
            end

            obj=obj@JumpFlowSystem(Ac, Bwc, Ad, Bwd, Czc, Dzc_wc, Czd, Dzd_wd);

            % Make sure the matrices have the appropriate sizes (the
            % matrices used above are checked in the constructor of
            % SampledDataSystem

            % Check controller output to state matrices
            nx = size(Ac, 1);
            nu = size(Buc, 2);

            if nx ~= size(Buc, 1)
                error('The number of rows of "Buc" does not match the state dimension');
            end

            if nx ~= size(Bud, 1)
                error('The number of rows of "Bud" does not match the state dimension');
            end

            if nu ~= size(Bud, 2)
                error('The number of columns of "Bud" does not match the controller output dimension');
            end

            % Check controller output to performance channels feed-through matrix
            nzc = size(Czc, 1);
            nzd = size(Czd, 1);

            if nzc ~= size(Dzc_uc, 1)
                error('The number of rows of "Dzd_uc" does not match the amount of discrete-time performance channels');
            end

            if nu ~= size(Dzc_uc, 2)
                error('The number of columns of "Dzd_uc" does not match the controller output dimension');
            end

            if nzd ~= size(Dzd_ud, 1)
                error('The number of rows of "Dzd_ud" does not match the amount of discrete-time performance channels');
            end

            if nu ~= size(Dzd_ud, 2)
                error('The number of columns of "Dzd_ud" does not match the controller output dimension');
            end

            % Check controller input matrices
            if nx ~= size(Cy, 2)
                error('The number of columns of "Cy" does not match the state dimension');
            end

            ny = size(Cy, 1);

            if ny ~= size(Dy_wd, 1)
                error('The number of rows of "Dy_wd" does not match the amount of controller input dimension');
            end

            nwd = size(Bwd, 2);

            if nwd ~= size(Dy_wd, 2)
                error('The number of columns of "Dy_wd" does not match the the amount of discrete-time disturbance channels');
            end

            if ny ~= size(Dy_ud, 1)
                error('The number of rows of "Dy_ud" does not match the controller input dimension');
            end

            if nu ~= size(Dy_ud, 2)
                error('The number of columns of "Dy_ud" does not match the controller output dimension');
            end

            % Controller output to state matrices
            obj.Buc = Buc;
            obj.Bud = Bud;

            % Controller output to performance channels feed-through matrices
            obj.Dzc_uc = Dzc_uc;
            obj.Dzd_ud = Dzd_ud;

            % Controller input matrices
            obj.Cy = Cy;
            obj.Dy_wd = Dy_wd;
            obj.Dy_ud = Dy_ud;

            % Set Loop to "Open"
            obj.Loop = 'Open';
        end

        %% Dimensions
        % When OpenLoopSampledDataSystem.nu is called it is calculated based on the
        % Buc property
        function nu = get.nu(OpenLoopSampledDataSystem)
            nu = size(OpenLoopSampledDataSystem.Bud, 2);
        end

        % When OpenLoopSampledDataSystem.ny is called it is calculated based on the
        % Cy property
        function ny = get.ny(OpenLoopSampledDataSystem)
            ny = size(OpenLoopSampledDataSystem.Cy, 1);
        end

        % Check dimensions of every state-space matrix of the open-loop
        % sampled-data system
        function dimCheck(OLSDSystem)
            arguments
                OLSDSystem (1,1) OpenLoopSampledDataSystem
            end

            % Define dimensions
            nx = OLSDSystem.nx;
            nwc = OLSDSystem.nwc;
            nwd = OLSDSystem.nwd;
            nzc = OLSDSystem.nzc;
            nzd = OLSDSystem.nzd;
            nu = OLSDSystem.nu;
            ny = OLSDSystem.ny;

            % Check state dimension
            if nx ~= size(OLSDSystem.Ac, 2)
                error('The flow state matrix "Ac" must be square');
            end

            if nx ~= size(OLSDSystem.Bwc, 1)
                error('The number of rows of "Bwc" does not match the state dimension');
            end

            if nx ~= size(OLSDSystem.Buc, 1)
                error('The number of rows of "Buc" does not match the state dimension');
            end

            if nx ~= size(OLSDSystem.Ad, 1) || nx ~= size(OLSDSystem.Ad, 2)
                error('The dimensions of "Ac" do not match the dimensions of "Ad"');
            end

            if nx ~= size(OLSDSystem.Bwd, 1)
                error('The number of rows of "Bwd" does not match the state dimension');
            end

            if nx ~= size(OLSDSystem.Bud, 1)
                error('The number of rows of "Bud" does not match the state dimension');
            end

            if nx ~= size(OLSDSystem.Czc, 2)
                error('The number of columns of "Czc" does not match the state dimension');
            end

            if nx ~= size(OLSDSystem.Czd, 2)
                error('The number of columns of "Czd" does not match the state dimension');
            end

            if nx ~= size(OLSDSystem.Cy, 2)
                error('The number of columns of "Cy" does not match the state dimension');
            end

            % Check continuous-time disturbance channels
            if nwc ~= size(OLSDSystem.Dzc_wc, 2)
                error('The number of columns of "Dzc_wc" does not match the amount of continuous-time disturbance channels');
            end

            % Check discrete-time disturbance channels
            if nwd ~= size(OLSDSystem.Dzd_wd, 2)
                error('The number of columns of "Dzd_wd" does not match the amount of continuous-time disturbance channels');
            end

            if nwd ~= size(OLSDSystem.Dy_wd, 2)
                error('The number of columns of "Dyd_wd" does not match the amount of continuous-time disturbance channels');
            end

            % Check continuous-time performance channels
            if nzc ~= size(OLSDSystem.Dzc_wc, 1)
                error('The number of rows of "Dzc_wc" does not match the amount of continuous-time performance channels');
            end

            % Check discrete-time performance channels
            if nzd ~= size(OLSDSystem.Dzd_wd, 1)
                error('The number of rows of "Dzd_wd" does not match the amount of discrete-time performance channels');
            end

            if nzd ~= size(OLSDSystem.Dzd_ud, 1)
                error('The number of rows of "Dzd_ud" does not match the amount of discrete-time performance channels');
            end

            % Check controller output
            if nu ~= size(OLSDSystem.Bud, 2)
                error('The number of columns of "Bud" does not match the controller output dimension');
            end

            if nu ~= size(OLSDSystem.Dzc_uc, 2)
                error('The number of columns of "Dzc_uc" does not match the controller output dimension');
            end

            if nu ~= size(OLSDSystem.Dzd_ud, 2)
                error('The number of columns of "Dzd_ud" does not match the controller output dimension');
            end

            if nu ~= size(OLSDSystem.Dy_ud, 2)
                error('The number of columns of "Dy_ud" does not match the controller output dimension');
            end

            % Check controller input
            if ny ~= size(OLSDSystem.Dy_wd, 1)
                error('The number of rows of "Dy_wd" does not match the controller input dimension');
            end

            if ny ~= size(OLSDSystem.Dy_ud, 1)
                error('The number of rows of "Dy_ud" does not match the controller input dimension');
            end
        end

        %% Operator overloading
        % Override the uplus operator for OpenLoopSampledDataSystem class object
        SDSystem = uplus(objSD)

        % Override the uminus operator for OpenLoopSampledDataSystem class object
        SDSystem = uminus(objSD)

        % Override the plus operator for OpenLoopSampledDataSystem class object
        SDSystem = plus(objSD1, objSD2)

        % Override the minus operator for OpenLoopSampledDataSystem class object
        SDSystem = minus(objSD1, objSD2)

        % Override the mtimes operator for OpenLoopSampledDataSystem class object
        SDSystem = mtimes(obj1, obj2)

        % Override the horzcat operator for OpenLoopSampledDataSystem class object
        SDSystem = horzcat(obj1, obj2)

        % Override the vertcat operator for OpenLoopSampledDataSystem class object
        SDSystem = vertcat(obj1, obj2)

        %% Sampled-data specific methods
        % Add weighting filters to the disturbance and performance channels
        weightedSD = appendWeightingFilters(objSD, Vc, Vd, Wc, Wd)

        % Calculate closed-loop jump-flow system based on the
        % interconnection
        objJF = lft(OLSDSystem1, OLSDSystem2, opts)

        objSD = addCompensator(OLSDSystem, compensator, opts)

        % Determine the state-space model of the sampled-data system when a
        % reconstructor is applied to the controller output
        fobjSD_reconstructed = applyReconstructor(objSD, opts)

        % Perform analysis for various system gains and norms
        normValue = analysis(OLSDSystem, performanceIndicator, opts)

        % Determine closed-loop flow matrices based an open-loop sampled-data
        % system
        [Ac, Bwc, Czc, Dzc_wc] = ClosedLoopFlowMatrices(OLSDSystem, opts, nc)

        % Perform synthesis for various system gains and norms
        [Controller, synthesisNormValue, CLJFSystem] = synthesis(OLSDSystem, performanceIndicator, opts)
    end
    methods (Access = protected)
        function propgrp = getPropertyGroups(~)
            proplist = {'Ac', 'Bwc', 'Buc', 'Ad', 'Bwd', 'Bud', 'Czc', 'Dzc_wc', 'Dzc_uc', 'Czd', 'Dzd_wd', 'Dzd_ud','Cy','Dy_wd', 'Dy_ud'};
            propgrp = matlab.mixin.util.PropertyGroup(proplist);
        end
    end
end