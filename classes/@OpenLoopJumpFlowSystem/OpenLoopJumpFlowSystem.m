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
    %  \dot{x} = Ac*x   + Bwc*w_c     + Buf*u_f
    %   x^+    = Ad*x   + Bwd*w_d     + Buj*u_j
    %   z_c    = Czc*x  + Dzc_wc*w_c  + Dzc_uf*u_f
    %   z_d    = Czd*x  + Dzd_wd*w_d  + Dzd_uj*u_j
    %   y_f    = Cyc*x  + Dyc_wd*w_d  + Dyc_uf*u_f
    %   y_j    = Cyd*x  + Dyd_wd*w_d  + Dyd_uj*u_j

    properties
        %Controller output to state matrices
        Buf     double {mustBeFinite(Buf)}
        Buj     double {mustBeFinite(Buj)}

        %Controller output to performance channel feed-through terms
        Dzc_uf  double {mustBeFinite(Dzc_uf)}
        Dzd_uj  double {mustBeFinite(Dzd_uj)}

        %Controller input matrices
        Cyf     double {mustBeFinite(Cyf)}
        Dyf_wc  double {mustBeFinite(Dyf_wc)}
        Dyf_uf  double {mustBeFinite(Dyf_uf)}
        Cyj     double {mustBeFinite(Cyj)}
        Dyj_wd  double {mustBeFinite(Dyj_wd)}
        Dyj_uj  double {mustBeFinite(Dyj_uj)}
    end

    properties (Dependent = true, SetAccess = private, GetAccess = 'public')
        nuf     (1,1) double
        nuj     (1,1) double
        nyf     (1,1) double
        nyj     (1,1) double
    end

    methods
        %% Constructor
        function obj = OpenLoopJumpFlowSystem(Ac, Bwc, Buf, Ad, Bwd, Buj, Czc, Dzc_wc, Dzc_uf, Czd, Dzd_wd, Dzd_uj, Cyf, Dyf_wc, Dyf_uf, Cyj, Dyj_wd, Dyj_uj)

            % Here the arguments is setup such that the input arguments are
            % variable. That means that this constructor accepts 0 up to 15
            % input arguments. However, no more than 15 input arguments is
            % allowed.
            arguments
                Ac = []
                Bwc = zeros(size(Ac, 1), 0)
                Buf = zeros(size(Ac, 1), 0)
                Ad = eye(size(Ac))
                Bwd = zeros(size(Ac, 1), 0)
                Buj = zeros(size(Ac, 1), 0)
                Czc = zeros(0, size(Ac, 1))
                Dzc_wc = zeros(size(Czc, 1), size(Bwc, 2))
                Dzc_uf = zeros(size(Czc, 1), size(Buf, 2))
                Czd = zeros(0, size(Ac, 1))
                Dzd_wd = zeros(size(Czd, 1), size(Bwd, 2))
                Dzd_uj = zeros(size(Czd, 1), size(Buj, 2))
                Cyf = zeros(0, size(Ac, 1))
                Dyf_wc = zeros(size(Cyf, 1), size(Bwc, 2))
                Dyf_uf = zeros(size(Cyf, 1), size(Buf, 2))
                Cyj = zeros(0, size(Ac, 1))
                Dyj_wd = zeros(size(Cyj, 1), size(Bwd, 2))
                Dyj_uj = zeros(size(Cyj, 1), size(Buj, 2))

            end

            obj=obj@JumpFlowSystem(Ac, Bwc, Ad, Bwd, Czc, Dzc_wc, Czd, Dzd_wd);

            % Make sure the matrices have the appropriate sizes (the
            % matrices used above are checked in the constructor of
            % SampledDataSystem

            % Check controller output to state matrices
            nx = size(Ac, 1);

            if nx ~= size(Buf, 1)
                error('The number of rows of "Buc" does not match the state dimension');
            end

            if nx ~= size(Buj, 1)
                error('The number of rows of "Bud" does not match the state dimension');
            end

            % Check controller output to discrete-time performance channels feed-through matrix
            nzc = size(Czc, 1);
            nzd = size(Czd, 1);

            if nzc ~= size(Dzc_uf, 1)
                error('The number of rows of "Dzc_uc" does not match the amount of continuous-time performance channels');
            end

            if nzd ~= size(Dzd_uj, 1)
                error('The number of rows of "Dzd_ud" does not match the amount of discrete-time performance channels');
            end

            nud = size(Buj, 2);
            nuc = size(Buf, 2);

            if nuc ~= size(Dzc_uf, 2)
                error('The number of columns of "Dzd_uc" does not match the continuous-time controller output dimension');
            end

            if nud ~= size(Dzd_uj, 2)
                error('The number of columns of "Dzd_ud" does not match the discrete-time controller output dimension');
            end

            % Check controller input matrices
            if nx ~= size(Cyf, 2)
                error('The number of columns of "Cyc" does not match the state dimension');
            end

            if nx ~= size(Cyj, 2)
                error('The number of columns of "Cyd" does not match the state dimension');
            end

            nyc = size(Cyf, 1);
            nyd = size(Cyj, 1);

            if nyc ~= size(Dyf_wc, 1)
                error('The number of rows of "Dyc_wc" does not match the continuous-time controller input dimension');
            end

            if nyd ~= size(Dyj_wd, 1)
                error('The number of rows of "Dyd_wd" does not match the discrete-time controller input dimension');
            end

            nwc = size(Bwc, 2);
            nwd = size(Bwd, 2);

            if nwc ~= size(Dyf_wc, 2)
                error('The number of columns of "Dyc_wc" does not match the amount of continuous-time disturbance channels');
            end

            if nwd ~= size(Dyj_wd, 2)
                error('The number of columns of "Dyd_wd" does not match the amount of discrete-time disturbance channels');
            end

            if nyc ~= size(Dyf_uf, 1)
                error('The number of rows of "Dyc_uc" does not match the continuous-time controller input dimension');
            end

            if nuc ~= size(Dyf_uf, 2)
                error('The number of columns of "Dyc_uc" does not match the continuous-time controller output dimension');
            end

            if nyd ~= size(Dyj_uj, 1)
                error('The number of rows of "Dyd_ud" does not match the discrete-time controller input dimension');
            end

            if nud ~= size(Dyj_uj, 2)
                error('The number of columns of "Dyd_ud" does not match the discrete-time controller output dimension');
            end

            % Controller output to state matrix
            obj.Buf = Buf;
            obj.Buj = Buj;

            % Controller output to performance channels feed-through matrix
            obj.Dzc_uf = Dzc_uf;
            obj.Dzd_uj = Dzd_uj;

            % Controller input matrices
            obj.Cyf = Cyf;
            obj.Dyf_wc = Dyf_wc;
            obj.Dyf_uf = Dyf_uf;
            obj.Cyj = Cyj;
            obj.Dyj_wd = Dyj_wd;
            obj.Dyj_uj = Dyj_uj;

            % Set Loop to "Open"
            obj.Loop = 'Open';
        end

        %% Dimensions
        % When OpenLoopJumpFlowSystem.nuc is called it is calculated based on the
        % Buc property
        function nuc = get.nuf(OpenLoopJumpFlowSystem)
            nuc = size(OpenLoopJumpFlowSystem.Buf, 2);
        end

        % When OpenLoopJumpFlowSystem.nud is called it is calculated based on the
        % Bud property
        function nud = get.nuj(OpenLoopJumpFlowSystem)
            nud = size(OpenLoopJumpFlowSystem.Buj, 2);
        end

        % When OpenLoopJumpFlowSystem.nyc is called it is calculated based on the
        % Cyc property
        function nyc = get.nyf(OpenLoopJumpFlowSystem)
            nyc = size(OpenLoopJumpFlowSystem.Cyf, 1);
        end

        % When OpenLoopJumpFlowSystem.nyd is called it is calculated based on the
        % Cydd property
        function nyd = get.nyj(OpenLoopJumpFlowSystem)
            nyd = size(OpenLoopJumpFlowSystem.Cyj, 1);
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
            nuc = OLJFSystem.nuf;
            nud = OLJFSystem.nuj;
            nyc = OLJFSystem.nyf;
            nyd = OLJFSystem.nyj;

            % Check state dimension
            if nx ~= size(OLJFSystem.Ac, 2)
                error('The flow state matrix "Ac" must be square');
            end

            if nx ~= size(OLJFSystem.Bwc, 1)
                error('The number of rows of "Bwc" does not match the state dimension');
            end

            if nx ~= size(OLJFSystem.Buf, 1)
                error('The number of rows of "Buc" does not match the state dimension');
            end

            if nx ~= size(OLJFSystem.Ad, 1) || nx ~= size(OLJFSystem.Ad, 2)
                error('The dimensions of "Ac" do not match the dimensions of "Ad"');
            end

            if nx ~= size(OLJFSystem.Bwd, 1)
                error('The number of rows of "Bwd" does not match the state dimension');
            end

            if nx ~= size(OLJFSystem.Buj, 1)
                error('The number of rows of "Bud" does not match the state dimension');
            end

            if nx ~= size(OLJFSystem.Czc, 2)
                error('The number of columns of "Czc" does not match the state dimension');
            end

            if nx ~= size(OLJFSystem.Czd, 2)
                error('The number of columns of "Czd" does not match the state dimension');
            end

            if nx ~= size(OLJFSystem.Cyf, 2)
                error('The number of columns of "Cyc" does not match the state dimension');
            end

            if nx ~= size(OLJFSystem.Cyj, 2)
                error('The number of columns of "Cyd" does not match the state dimension');
            end

            % Check continuous-time disturbance channels
            if nwc ~= size(OLJFSystem.Dzc_wc, 2)
                error('The number of columns of "Dzc_wc" does not match the amount of continuous-time disturbance channels');
            end

            if nwc ~= size(OLJFSystem.Dyf_wc, 2)
                error('The number of columns of "Dyc_wc" does not match the amount of continuous-time disturbance channels');
            end

            % Check discrete-time disturbance channels
            if nwd ~= size(OLJFSystem.Dzd_wd, 2)
                error('The number of columns of "Dzd_wd" does not match the amount of continuous-time disturbance channels');
            end

            if nwd ~= size(OLJFSystem.Dyj_wd, 2)
                error('The number of columns of "Dyd_wd" does not match the amount of continuous-time disturbance channels');
            end

            % Check continuous-time performance channels
            if nzc ~= size(OLJFSystem.Dzc_wc, 1)
                error('The number of rows of "Dzc_wc" does not match the amount of continuous-time performance channels');
            end

            if nzc ~= size(OLJFSystem.Dzc_uf, 1)
                error('The number of rows of "Dzc_uc" does not match the amount of continuous-time performance channels');
            end

            % Check discrete-time performance channels
            if nzd ~= size(OLJFSystem.Dzd_wd, 1)
                error('The number of rows of "Dzd_wd" does not match the amount of discrete-time performance channels');
            end

            if nzd ~= size(OLJFSystem.Dzd_uj, 1)
                error('The number of rows of "Dzd_ud" does not match the amount of discrete-time performance channels');
            end

            % Check continuous-time controller output
            if nuc ~= size(OLJFSystem.Dzc_uf, 2)
                error('The number of columns of "Dzc_uc" does not match the continuous-time controller output dimension');
            end

            if nuc ~= size(OLJFSystem.Dyf_uf, 2)
                error('The number of columns of "Dyc_uc" does not match the continuous-time controller output dimension');
            end

            % Check discrete-time controller output
            if nud ~= size(OLJFSystem.Dzd_uj, 2)
                error('The number of columns of "Dzd_ud" does not match the discrete-time controller output dimension');
            end

            if nud ~= size(OLJFSystem.Dyj_uj, 2)
                error('The number of columns of "Dyd_ud" does not match the discrete-time controller output dimension');
            end

            % Check continuous-time controller input
            if nyc ~= size(OLJFSystem.Dyf_wc, 1)
                error('The number of rows of "Dyc_wc" does not match the continuous-time controller input dimension');
            end

            if nyc ~= size(OLJFSystem.Dyf_uf, 1)
                error('The number of rows of "Dyc_uc" does not match the continuous-time controller input dimension');
            end

            % Check discrete-time controller input
            if nyd ~= size(OLJFSystem.Dyj_wd, 1)
                error('The number of rows of "Dyd_wd" does not match the discrete-time controller input dimension');
            end

            if nyd ~= size(OLJFSystem.Dyj_uj, 1)
                error('The number of rows of "Dyd_ud" does not match the discrete-time controller input dimension');
            end
        end

        %% OpenLoopJumpFlowSystem specific methods
        % Add weighting filters to the disturbance and performance channels
        function objJF = appendWeightingFilters(OLJFSystem, Wwc, Wwd, Wzc, Wzd)
            arguments
                OLJFSystem  (1,1) OpenLoopJumpFlowSystem
                Wwc         {mustBeNumericOrListedType(Wwc, "ss", "tf")} = 1
                Wwd         {mustBeNumericOrListedType(Wwd, "ss", "tf")} = 1
                Wzc         {mustBeNumericOrListedType(Wzc, "ss", "tf")} = 1
                Wzd         {mustBeNumericOrListedType(Wzd, "ss", "tf")} = 1
            end

            dimCheck(OLJFSystem);
            objSD = OpenLoopSampledDataSystem(OLJFSystem.Ac, OLJFSystem.Bwc, OLJFSystem.Buf, OLJFSystem.Ad, OLJFSystem.Bwd, OLJFSystem.Buj, OLJFSystem.Czc, OLJFSystem.Dzc_wc, ...
                                              OLJFSystem.Dzc_uf, OLJFSystem.Czd, OLJFSystem.Dzd_wd, OLJFSystem.Dzd_uj, OLJFSystem.Cyj, OLJFSystem.Dyj_wd, OLJFSystem.Dyj_uj);
            objSD = objSD.appendWeightingFilters(Wwc, Wwd, Wzc, Wzd);

            % Convert weighting filters which are possibly numeric or tf to
            % ss
            Wwc = minreal(Wwc, [], false);
            Wwd = minreal(Wwd, [], false);
            Wzc = minreal(Wzc, [], false);
            Wzd = minreal(Wzd, [], false);

            % State dimensions
            nx_wd = size(Wwd.A, 1);
            nx_zc = size(Wzc.A, 1);
            nx_zd = size(Wzd.A, 1);

            % Continuous-time controller input matrices
            Cyc = [OLJFSystem.Cyf, OLJFSystem.Dyf_wc*Wwc.C, zeros(OLJFSystem.nyf, nx_wd+nx_zc+nx_zd)];
            Dyc_wc = OLJFSystem.Dyf_wc*Wwc.D;
            Dyc_uc = OLJFSystem.Dyf_uf;

            % Construct OLJF object
            objJF = OpenLoopJumpFlowSystem(objSD.Ac, objSD.Bwc, objSD.Buc, objSD.Ad, objSD.Bwd, objSD.Bud, objSD.Czc, objSD.Dzc_wc, objSD.Dzc_uc, objSD.Czd, objSD.Dzd_wd, objSD.Dzd_ud, ...
                                           Cyc, Dyc_wc, Dyc_uc, objSD.Cy, objSD.Dy_wd, objSD.Dy_ud);
        end
    end
    methods (Access = protected)
        function propgrp = getPropertyGroups(~)
            proplist = {'Ac', 'Bwc', 'Buf', 'Ad', 'Bwd', 'Buj', 'Czc', 'Dzc_wc', 'Dzc_uf', 'Czd', 'Dzd_wd', 'Dzd_uj', 'Cyf','Dyf_wc', 'Dyf_uf', 'Cyj','Dyj_wd', 'Dyj_uj'};
            propgrp = matlab.mixin.util.PropertyGroup(proplist);
        end
    end
end