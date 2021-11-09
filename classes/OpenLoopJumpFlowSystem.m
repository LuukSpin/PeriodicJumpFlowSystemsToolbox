classdef OpenLoopJumpFlowSystem < JumpFlowSystem
    %OpenLoopJumpFlowSystem Construct an instance of this class
    %   OpenLoopJumpFlowSystem(Ac, Bwc, Buc, Ad, Bwd, Bud, Czc, Dzc_wc,
    %   Dzc_uc, Czd, Dzd_Wd, Dzd_u, Cyc, Dyc_wc, Dyc_uc, Cyd, Dyd_wd, Dyd_ud)
    %   construct an open-loop jump-flow system. A jump-flow system
    %   is sampled-data where discrete-time the controller output
    %   is connected to the continuous-time plant through a
    %   Zero-Order-Hold. This system has the following state-space
    %   realization:
    %
    %  \dot{x} = Ac*x   + Bwc*w_c     + Buc*u
    %   x^+    = Ad*x   + Bwd*w_d     + Bud*\hat{u}
    %   z_c    = Czc*x  + Dzc_wc*w_c  + Dzc_u*u
    %   z_d    = Czd*x  + Dzd_wd*w_d  + Dzd_u*\hat{u}
    %   y_c    = Cyc*x  + Dyc_wd*w_d  + Dyc_u*\hat{u}
    %   y_d    = Cyd*x  + Dyd_wd*w_d  + Dyd_u*\hat{u}

    properties
        %Controller output to state matrices
        Buc     double {mustBeFinite(Buc)}
        Bud     double {mustBeFinite(Bud)}

        %Controller output to performance channel feed-through terms
        Dzc_uc  double {mustBeFinite(Dzc_uc)}
        Dzd_ud  double {mustBeFinite(Dzd_ud)}

        %Controller input matrices
        Cyc     double {mustBeFinite(Cyc)}
        Dyc_wc  double {mustBeFinite(Dyc_wc)}
        Dyc_uc  double {mustBeFinite(Dyc_uc)}
        Cyd     double {mustBeFinite(Cyd)}
        Dyd_wd  double {mustBeFinite(Dyd_wd)}
        Dyd_ud  double {mustBeFinite(Dyd_ud)}
    end

    properties (Dependent = true, SetAccess = private, GetAccess = 'public')
        nuc     (1,1) double
        nud     (1,1) double
        nyc     (1,1) double
        nyd     (1,1) double
    end

    methods
        %% Constructor
        function obj = OpenLoopJumpFlowSystem(Ac, Bwc, Buc, Ad, Bwd, Bud, Czc, Dzc_wc, Dzc_uc, Czd, Dzd_wd, Dzd_ud, Cyc, Dyc_wc, Dyc_uc, Cyd, Dyd_wd, Dyd_ud)

            % Here the arguments is setup such that the input arguments are
            % variable. That means that this constructor accepts 0 up to 15
            % input arguments. However, no more than 15 input arguments is
            % allowed.
            arguments
                Ac = []
                Bwc = zeros(size(Ac, 1), 0)
                Buc = zeros(size(Ac, 1), 0)
                Ad = eye(size(Ac))
                Bwd = zeros(size(Ac, 1), 0)
                Bud = zeros(size(Ac, 1), 0)
                Czc = zeros(0, size(Ac, 1))
                Dzc_wc = zeros(size(Czc, 1), size(Bwc, 2))
                Dzc_uc = zeros(size(Czc, 1), size(Buc, 2))
                Czd = zeros(0, size(Ac, 1))
                Dzd_wd = zeros(size(Czd, 1), size(Bwd, 2))
                Dzd_ud = zeros(size(Czd, 1), size(Bud, 2))
                Cyc = zeros(0, size(Ac, 1))
                Dyc_wc = zeros(size(Cyc, 1), size(Bwc, 2))
                Dyc_uc = zeros(size(Cyc, 1), size(Buc, 2))
                Cyd = zeros(0, size(Ac, 1))
                Dyd_wd = zeros(size(Cyd, 1), size(Bwd, 2))
                Dyd_ud = zeros(size(Cyd, 1), size(Bud, 2))

            end

            obj=obj@JumpFlowSystem(Ac, Bwc, Ad, Bwd, Czc, Dzc_wc, Czd, Dzd_wd);

            % Make sure the matrices have the appropriate sizes (the
            % matrices used above are checked in the constructor of
            % SampledDataSystem

            % Check controller output to state matrices
            nx = size(Ac, 1);

            if nx ~= size(Buc, 1)
                error('The number of rows of "Buc" does not match the state dimension');
            end

            if nx ~= size(Bud, 1)
                error('The number of rows of "Bud" does not match the state dimension');
            end

            % Check controller output to discrete-time performance channels feed-through matrix
            nzc = size(Czc, 1);
            nzd = size(Czd, 1);

            if nzc ~= size(Dzc_uc, 1)
                error('The number of rows of "Dzc_uc" does not match the amount of continuous-time performance channels');
            end

            if nzd ~= size(Dzd_ud, 1)
                error('The number of rows of "Dzd_ud" does not match the amount of discrete-time performance channels');
            end

            nud = size(Bud, 2);
            nuc = size(Buc, 2);

            if nuc ~= size(Dzc_uc, 2)
                error('The number of columns of "Dzd_uc" does not match the continuous-time controller output dimension');
            end

            if nud ~= size(Dzd_ud, 2)
                error('The number of columns of "Dzd_ud" does not match the discrete-time controller output dimension');
            end

            % Check controller input matrices
            if nx ~= size(Cyc, 2)
                error('The number of columns of "Cyc" does not match the state dimension');
            end

            if nx ~= size(Cyd, 2)
                error('The number of columns of "Cyd" does not match the state dimension');
            end

            nyc = size(Cyc, 1);
            nyd = size(Cyd, 1);

            if nyc ~= size(Dyc_wc, 1)
                error('The number of rows of "Dyc_wc" does not match the continuous-time controller input dimension');
            end

            if nyd ~= size(Dyd_wd, 1)
                error('The number of rows of "Dyd_wd" does not match the discrete-time controller input dimension');
            end

            nwc = size(Bwc, 2);
            nwd = size(Bwd, 2);

            if nwc ~= size(Dyc_wc, 2)
                error('The number of columns of "Dyc_wc" does not match the amount of continuous-time disturbance channels');
            end

            if nwd ~= size(Dyd_wd, 2)
                error('The number of columns of "Dyd_wd" does not match the amount of discrete-time disturbance channels');
            end

            if nyc ~= size(Dyc_uc, 1)
                error('The number of rows of "Dyc_uc" does not match the continuous-time controller input dimension');
            end

            if nuc ~= size(Dyc_uc, 2)
                error('The number of columns of "Dyc_uc" does not match the continuous-time controller output dimension');
            end

            if nyd ~= size(Dyd_ud, 1)
                error('The number of rows of "Dyd_ud" does not match the discrete-time controller input dimension');
            end

            if nud ~= size(Dyd_ud, 2)
                error('The number of columns of "Dyd_ud" does not match the discrete-time controller output dimension');
            end

            % Controller output to state matrix
            obj.Buc = Buc;
            obj.Bud = Bud;

            % Controller output to performance channels feed-through matrix
            obj.Dzc_uc = Dzc_uc;
            obj.Dzd_ud = Dzd_ud;

            % Controller input matrices
            obj.Cyc = Cyc;
            obj.Dyc_wc = Dyc_wc;
            obj.Dyc_uc = Dyc_uc;
            obj.Cyd = Cyd;
            obj.Dyd_wd = Dyd_wd;
            obj.Dyd_ud = Dyd_ud;

            % Set Loop to "Open"
            obj.Loop = 'Open';
        end

        %% Dimensions
        % When OpenLoopJumpFlowSystem.nuc is called it is calculated based on the
        % Buc property
        function nuc = get.nuc(OpenLoopJumpFlowSystem)
            nuc = size(OpenLoopJumpFlowSystem.Buc, 2);
        end

        % When OpenLoopJumpFlowSystem.nud is called it is calculated based on the
        % Bud property
        function nud = get.nud(OpenLoopJumpFlowSystem)
            nud = size(OpenLoopJumpFlowSystem.Bud, 2);
        end

        % When OpenLoopJumpFlowSystem.nyc is called it is calculated based on the
        % Cyc property
        function nyc = get.nyc(OpenLoopJumpFlowSystem)
            nyc = size(OpenLoopJumpFlowSystem.Cyc, 1);
        end

        % When OpenLoopJumpFlowSystem.nyd is called it is calculated based on the
        % Cydd property
        function nyd = get.nyd(OpenLoopJumpFlowSystem)
            nyd = size(OpenLoopJumpFlowSystem.Cyd, 1);
        end
    
        % Check dimensions of every state-space matrix of the open-loop
        % jump-flow system
        function dimCheck(OLJFSystem)
            arguments
                OLJFSystem (1,1) OpenLoopJumpFlowSystem
            end

            % Define dimensions
            nx = OLJFSystem.nx;
            nwc = OLJFSystem.nwc;
            nwd = OLJFSystem.nwd;
            nzc = OLJFSystem.nzc;
            nzd = OLJFSystem.nzd;
            nuc = OLJFSystem.nuc;
            nud = OLJFSystem.nud;
            nyc = OLJFSystem.nyc;
            nyd = OLJFSystem.nyd;

            % Check state dimension
            if nx ~= size(OLJFSystem.Ac, 2)
                error('The flow state matrix "Ac" must be square');
            end

            if nx ~= size(OLJFSystem.Bwc, 1)
                error('The number of rows of "Bwc" does not match the state dimension');
            end

            if nx ~= size(OLJFSystem.Buc, 1)
                error('The number of rows of "Buc" does not match the state dimension');
            end

            if nx ~= size(OLJFSystem.Ad, 1) || nx ~= size(OLJFSystem.Ad, 2)
                error('The dimensions of "Ac" do not match the dimensions of "Ad"');
            end

            if nx ~= size(OLJFSystem.Bwd, 1)
                error('The number of rows of "Bwd" does not match the state dimension');
            end

            if nx ~= size(OLJFSystem.Bud, 1)
                error('The number of rows of "Bud" does not match the state dimension');
            end

            if nx ~= size(OLJFSystem.Czc, 2)
                error('The number of columns of "Czc" does not match the state dimension');
            end

            if nx ~= size(OLJFSystem.Czd, 2)
                error('The number of columns of "Czd" does not match the state dimension');
            end

            if nx ~= size(OLJFSystem.Cyc, 2)
                error('The number of columns of "Cyc" does not match the state dimension');
            end

            if nx ~= size(OLJFSystem.Cyd, 2)
                error('The number of columns of "Cyd" does not match the state dimension');
            end

            % Check continuous-time disturbance channels
            if nwc ~= size(OLJFSystem.Dzc_wc, 2)
                error('The number of columns of "Dzc_wc" does not match the amount of continuous-time disturbance channels');
            end

            if nwc ~= size(OLJFSystem.Dyc_wc, 2)
                error('The number of columns of "Dyc_wc" does not match the amount of continuous-time disturbance channels');
            end

            % Check discrete-time disturbance channels
            if nwd ~= size(OLJFSystem.Dzd_wd, 2)
                error('The number of columns of "Dzd_wd" does not match the amount of continuous-time disturbance channels');
            end

            if nwd ~= size(OLJFSystem.Dyd_wd, 2)
                error('The number of columns of "Dyd_wd" does not match the amount of continuous-time disturbance channels');
            end

            % Check continuous-time performance channels
            if nzc ~= size(OLJFSystem.Dzc_wc, 1)
                error('The number of rows of "Dzc_wc" does not match the amount of continuous-time performance channels');
            end

            if nzc ~= size(OLJFSystem.Dzc_uc, 1)
                error('The number of rows of "Dzc_uc" does not match the amount of continuous-time performance channels');
            end

            % Check discrete-time performance channels
            if nzd ~= size(OLJFSystem.Dzd_wd, 1)
                error('The number of rows of "Dzd_wd" does not match the amount of discrete-time performance channels');
            end

            if nzd ~= size(OLJFSystem.Dzd_ud, 1)
                error('The number of rows of "Dzd_ud" does not match the amount of discrete-time performance channels');
            end

            % Check continuous-time controller output
            if nuc ~= size(OLJFSystem.Dzc_uc, 2)
                error('The number of columns of "Dzc_uc" does not match the continuous-time controller output dimension');
            end

            if nuc ~= size(OLJFSystem.Dyc_uc, 2)
                error('The number of columns of "Dyc_uc" does not match the continuous-time controller output dimension');
            end

            % Check discrete-time controller output
            if nud ~= size(OLJFSystem.Dzd_ud, 2)
                error('The number of columns of "Dzd_ud" does not match the discrete-time controller output dimension');
            end

            if nud ~= size(OLJFSystem.Dyd_ud, 2)
                error('The number of columns of "Dyd_ud" does not match the discrete-time controller output dimension');
            end

            % Check continuous-time controller input
            if nyc ~= size(OLJFSystem.Dyc_wc, 1)
                error('The number of rows of "Dyc_wc" does not match the continuous-time controller input dimension');
            end

            if nyc ~= size(OLJFSystem.Dyc_uc, 1)
                error('The number of rows of "Dyc_uc" does not match the continuous-time controller input dimension');
            end

            % Check discrete-time controller input
            if nyd ~= size(OLJFSystem.Dyd_wd, 1)
                error('The number of rows of "Dyd_wd" does not match the discrete-time controller input dimension');
            end

            if nyd ~= size(OLJFSystem.Dyd_ud, 1)
                error('The number of rows of "Dyd_ud" does not match the discrete-time controller input dimension');
            end
        end
    end
    methods (Access = protected)
        function propgrp = getPropertyGroups(~)
            proplist = {'Loop', 'Ac', 'Bwc', 'Buc', 'Ad', 'Bwd', 'Bud', 'Czc', 'Dzc_wc', 'Dzc_uc', 'Czd', 'Dzd_wd', 'Dzd_ud', 'Cyc','Dyc_wc', 'Dyc_uc', 'Cyd','Dyd_wd', 'Dyd_ud'};
            propgrp = matlab.mixin.util.PropertyGroup(proplist);
        end
    end
end