classdef JumpFlowSystem < matlab.mixin.CustomDisplay
    %JumpFlowSystem is a constructor for this class
    %   JumpFlowSystem(Ac, Bwc, Ad, Bwd, Czc, Dzc_wc, Czd, Dzd_wd)
    %   constructs a closed-loop jump-flow system. This is a
    %   closed-loop sampled data system where a Zero-Order-Hold is
    %   used to connect the discrete-time controller output with
    %   the continuous-time plant. This system has the following
    %   state-space realization:
    %
    %  \dot{x} = Ac*x  + Bwc*w_c
    %   x^+    = Ad*x  + Bwd*w_d
    %   z_c    = Czc*x + Dzc_wc*w_c
    %   z_d    = Czd*x + Dzd_wd*w_d

    properties (SetAccess = private)
        %Flow matrices
        Ac      double {mustBeFinite(Ac)}
        Bwc     double {mustBeFinite(Bwc)}

        %Jump Matrices
        Ad      double {mustBeFinite(Ad)}
        Bwd     double {mustBeFinite(Bwd)}

        %Continuous-time performance channel matrices
        Czc     double {mustBeFinite(Czc)}
        Dzc_wc  double {mustBeFinite(Dzc_wc)}

        %Discrete-time performance channel matrices
        Czd     double {mustBeFinite(Czd)}
        Dzd_wd  double {mustBeFinite(Dzd_wd)}
    end

    properties (Dependent = true, SetAccess = private, GetAccess = 'public')
        nx      (1,1) double
        nwc     (1,1) double
        nzc     (1,1) double
        nwd     (1,1) double
        nzd     (1,1) double
    end

    properties (GetAccess = 'public', SetAccess = {?OpenLoopJumpFlowSystem, ?OpenLoopSampledDataSystem})
        Loop = 'Closed'
    end

    methods
        %% Constructor
        function obj = JumpFlowSystem(Ac, Bwc, Ad, Bwd, Czc, Dzc_wc, Czd, Dzd_wd)

            % Here the arguments is setup such that the input arguments are
            % variable. That means that this constructor accepts 0 up to 8
            % input arguments. However, no more than 8 input arguments is
            % allowed.
            arguments
                Ac = []
                Bwc = zeros(size(Ac,1), 0)
                Ad = zeros(size(Ac))
                Bwd = zeros(size(Ac,1), 0)
                Czc = zeros(0, size(Ac, 1))
                Dzc_wc = zeros(size(Czc, 1), size(Bwc, 2))
                Czd = zeros(0, size(Ac, 1))
                Dzd_wd = zeros(size(Czd, 1), size(Bwd, 2))
            end

            % Make sure that the matrices have the appropriate sizes
            nx = size(Ac, 1);

            if nx ~= size(Ac, 2)
                error('The flow state matrix "Ac" must be square');
            end

            if nx ~= size(Bwc, 1)
                error('The number of rows of "Bwc" does not match the state dimension');
            end

            nwc = size(Bwc, 2);

            if nx ~= size(Ad, 1) || nx ~= size(Ad, 2)
                error('The dimensions of "Ac" do not match the dimensions of "Ad"');
            end

            if nx ~= size(Bwd, 1)
                error('The number of rows of "Bwd" does not match the state dimension');
            end

            nwd = size(Bwd, 2);

            if nx ~= size(Czc, 2)
                error('The number of columns of "Czc" does not match the state dimension');
            end

            nzc = size(Czc, 1);

            if nzc ~= size(Dzc_wc, 1)
                error('The number of rows of "Dzc_wc" does not match the amount of continous-time performance channels');
            end

            if nwc ~= size(Dzc_wc, 2)
                error('The number of columns of "Dzc_wc" does not match the amount of continuous-time disturbance channels');
            end

            if nx ~= size(Czd, 2)
                error('The number of columns of "Czd" does not match the state dimension');
            end

            nzd = size(Czd, 1);

            if nwd ~= size(Dzd_wd, 2)
                error('The number of columns of "Dzd_wd" does not match the amount of discrete-time disturbance channels');
            end

            if nzd ~= size(Dzd_wd, 1)
                error('The number of rows of "Dzd_wd" does not match the amount of discrete-time performance channels');
            end

            % Flow matrices
            obj.Ac = Ac;
            obj.Bwc = Bwc;

            % Jump matrices
            obj.Ad = Ad;
            obj.Bwd = Bwd;

            % Continuous-time performance channels matrices
            obj.Czc = Czc;
            obj.Dzc_wc = Dzc_wc;

            % Discrete-time performance channels matrices
            obj.Czd = Czd;
            obj.Dzd_wd = Dzd_wd;
        end

        %% Dimensions
        % When JumpFlowSystem.nx is called it is calculated based on the
        % Ac property
        function nx = get.nx(JumpFlowSystem)
            nx = size(JumpFlowSystem.Ac, 1);
        end

        % When JumpFlowSystem.nwc is called it is calculated based on the
        % Bwc property
        function nwc = get.nwc(JumpFlowSystem)
            nwc = size(JumpFlowSystem.Bwc, 2);
        end

        % When JumpFlowSystem.nzc is called it is calculated based on the
        % Czc property
        function nzc = get.nzc(JumpFlowSystem)
            nzc = size(JumpFlowSystem.Czc, 1);
        end

        % When JumpFlowSystem.nwd is called it is calculated based on the
        % Bwd property
        function nwd = get.nwd(JumpFlowSystem)
            nwd = size(JumpFlowSystem.Bwd, 2);
        end

        % When JumpFlowSystem.nzd is called it is calculated based on the
        % Czd property
        function nzd = get.nzd(JumpFlowSystem)
            nzd = size(JumpFlowSystem.Czd, 1);
        end

        % Check dimensions of every state-space matrix of the open-loop
        % sampled-data system
        function dimCheck(OLSDSystem)
            arguments
                OLSDSystem (1,1) JumpFlowSystem
            end

            % Define dimensions
            nx = OLSDSystem.nx;
            nwc = OLSDSystem.nwc;
            nwd = OLSDSystem.nwd;
            nzc = OLSDSystem.nzc;
            nzd = OLSDSystem.nzd;

            % Check state dimension
            if nx ~= size(OLSDSystem.Ac, 2)
                error('The flow state matrix "Ac" must be square');
            end

            if nx ~= size(OLSDSystem.Bwc, 1)
                error('The number of rows of "Bwc" does not match the state dimension');
            end

            if nx ~= size(OLSDSystem.Ad, 1) || nx ~= size(OLSDSystem.Ad, 2)
                error('The dimensions of "Ac" do not match the dimensions of "Ad"');
            end

            if nx ~= size(OLSDSystem.Bwd, 1)
                error('The number of rows of "Bwd" does not match the state dimension');
            end

            if nx ~= size(OLSDSystem.Czc, 2)
                error('The number of columns of "Czc" does not match the state dimension');
            end

            if nx ~= size(OLSDSystem.Czd, 2)
                error('The number of columns of "Czd" does not match the state dimension');
            end

            % Check continuous-time disturbance channels
            if nwc ~= size(OLSDSystem.Dzc_wc, 2)
                error('The number of columns of "Dzc_wc" does not match the amount of continuous-time disturbance channels');
            end

            % Check discrete-time disturbance channels
            if nwd ~= size(OLSDSystem.Dzd_wd, 2)
                error('The number of columns of "Dzd_wd" does not match the amount of continuous-time disturbance channels');
            end

            % Check continuous-time performance channels
            if nzc ~= size(OLSDSystem.Dzc_wc, 1)
                error('The number of rows of "Dzc_wc" does not match the amount of continuous-time performance channels');
            end

            % Check discrete-time performance channels
            if nzd ~= size(OLSDSystem.Dzd_wd, 1)
                error('The number of rows of "Dzd_wd" does not match the amount of discrete-time performance channels');
            end
        end

        %% Operator overloading
        % Override the uplus operator for JumpFlowSystem class object
        function JFSystem = uplus(objJF)
            arguments
                objJF (1,1) JumpFlowSystem
            end

            JFSystem = objJF;
        end

        % Override the uminus operator for JumpFlowSystem class object
        function JFSystem = uminus(objJF)
            arguments
                objJF (1,1) JumpFlowSystem
            end

            Ac = objJF.Ac;
            Bwc = objJF.Bwc;
            Ad = objJF.Ad;
            Bwd = objJF.Bwd;
            Czc = -objJF.Czc;
            Dzc_wc = -objJF.Dzc_wc;
            Czd = -objJF.Czd;
            Dzd_wd = -objJF.Dzd_wd;

            JFSystem = JumpFlowSystem(Ac, Bwc, Ad, Bwd, Czc, Dzc_wc, Czd, Dzd_wd);
        end

        % Override the plus operator for JumpFlowSystem class object
        function JFSystem = plus(obj1, obj2)
            arguments
                obj1 (1,1) JumpFlowSystem
                obj2 (1,1) JumpFlowSystem
            end

            Ac = blkdiag(obj1.Ac, obj2.Ac);
            Bwc = [obj1.Bwc; obj2.Bwc];
            Ad = blkdiag(obj1.Ad, obj2.Ad);
            Bwd = [obj1.Bwd; obj2.Bwd];
            Czc = [obj1.Czc, obj2.Czc];
            Dzc_wc = obj1.Dzc_wc+obj2.Dzc_wc;
            Czd = [obj1.Czd, obj2.Czd];
            Dzd_wd = obj1.Dzd_wd+obj2.Dzd_wd;

            JFSystem = JumpFlowSystem(Ac, Bwc, Ad, Bwd, Czc, Dzc_wc, Czd, Dzd_wd);
        end

        % Override the minus operator for JumpFlowSystem class object
        function JFSystem = minus(obj1, obj2)
            arguments
                obj1 (1,1) JumpFlowSystem
                obj2 (1,1) JumpFlowSystem
            end

            JFSystem = obj1 + (-obj2);
        end

        % Override the mtimes operator for JumpFlowSystem class object
        function JFSystem = mtimes(obj1, obj2)
        % MTIMES specifies the multiplication operator * for jump-flow
        % systems
        %   Multiplication of jump-flow systems specifies a series
        %   connection of system that looks like the following block
        %   diagram:
        %   
        %   
            arguments
                obj1 (1,1) JumpFlowSystem
                obj2 (1,1) JumpFlowSystem
            end

            nx1 = obj2.nx;
            nx2 = obj1.nx;
            nzc1 = obj2.nzc;
            nzd1 = obj2.nzd;
            nwc2 = obj1.nwc;
            nwd2 = obj1.nwd;

            if nzc1 ~= nwc2
                error('The amount of continuous-time performance channels of the first system is not equal to the amount of continuous-time disturbance channels of the second system.');
            end

            if nzd1 ~= nwd2
                error('The amount of discrete-time performance channels of the first system is not equal to the amount of discrete-time disturbance channels of the second system.');
            end

            Ac = [obj2.Ac, zeros(nx1, nx2); obj1.Bwc*obj2.Czc, obj1.Ac];
            Bwc = [obj2.Bwc; obj1.Bwc*obj2.Dzc_wc];
            Ad = [obj2.Ad, zeros(nx1, nx2); obj1.Bwd*obj2.Czd, obj1.Ad];
            Bwd = [obj2.Bwd; obj1.Bwd*obj2.Dzd_wd];

            Czc = [obj1.Dzc_wc*obj2.Czc, obj1.Czc];
            Dzc_wc = obj1.Dzc_wc*obj2.Dzc_wc;
            Czd = [obj1.Dzd_wd*obj2.Czd, obj1.Czd];
            Dzd_wd = obj1.Dzd_wd*obj2.Dzd_wd;

            JFSystem = JumpFlowSystem(Ac, Bwc, Ad, Bwd, Czc, Dzc_wc, Czd, Dzd_wd);

        end
        
        % Override the horzcat operator for JumpFlowSystem class object
        function JFSystem = horzcat(obj1, obj2)
            arguments
                obj1 (1,1) JumpFlowSystem
                obj2 (1,1) JumpFlowSystem
            end

            Ac = blkdiag(obj1.Ac, obj2.Ac);
            Bwc = blkdiag(obj1.Bwc, obj2.Bwc);
            Ad = blkdiag(obj1.Ad, obj2.Ad);
            Bwd = blkdiag(obj1.Bwd, obj2.Bwd);

            Czc = [obj1.Czc, obj2.Czc];
            Dzc_wc = [obj1.Dzc_wc, obj2.Dzc_wc];
            Czd = [obj1.Czd, obj2.Czd];
            Dzd_wd = [obj1.Dzd_wd, obj2.Dzd_wd];

            JFSystem = JumpFlowSystem(Ac, Bwc, Ad, Bwd, Czc, Dzc_wc, Czd, Dzd_wd);
        end

        % Override the vertcat operator for JumpFlowSystem class object
        function JFSystem = vertcat(obj1, obj2)
            arguments
                obj1 (1,1) JumpFlowSystem
                obj2 (1,1) JumpFlowSystem
            end

            Ac = blkdiag(obj1.Ac, obj2.Ac);
            Bwc = [obj1.Bwc; obj2.Bwc];
            Ad = blkdiag(obj1.Ad, obj2.Ad);
            Bwd = [obj1.Bwd; obj2.Bwd];

            Czc = blkdiag(obj1.Czc, obj2.Czc);
            Dzc_wc = [obj1.Dzc_wc; obj2.Dzc_wc];
            Czd = blkdiag(obj1.Czd, obj2.Czd);
            Dzd_wd = [obj1.Dzd_wd; obj2.Dzd_wd];

            JFSystem = JumpFlowSystem(Ac, Bwc, Ad, Bwd, Czc, Dzc_wc, Czd, Dzd_wd);
        end

        %% System theoretic properties

        % Stability function
        function stabilityFlag = isstable(objJF, opts)
            arguments
                objJF   JumpFlowSystem
                opts    jfopt
            end
            h = opts.simulation.SampleTime;
            stabilityFlag = all(abs(eig(expm(objJF.Ac*h)*objJF.Ad)) < 1);
        end

        % Reachability function
        function reachabilityFlag = isreachable(objJF, opts)
            arguments
                objJF   JumpFlowSystem
                opts    jfopt
            end

            Ac = objJF.Ac;
            Bwc = objJF.Bwc;
            Ad = objJF.Ad;
            Bwd = objJF.Bwd;

            nx = objJF.nx;

            monodromyMatrix = expm(Ac*h)*Ad;

            CTLTIctrbMat = ctrb(Ac, Bwc);
            Bd_bar = [Bwd Ad*CTLTIctrbMat];

            JFreachabilityMatrix = [ctrb(monodromyMatrix, Bd_bar) CTLTIctrbMat];
            [~, Sing] = svd(JFreachabilityMatrix);
            Sing = diag(Sing(1:nx, 1:nx));
            if all(Sing >= opts.LMI.numericalAccuracy)
                reachabilityFlag = true;
            else
                reachabilityFlag = false;
            end

        end

        % Strong reachability function
        function strongReachabilityFlag = isstrongreachable(objJF, opts)
        end

        % Controllability function
        function controllabilityFlag = iscontrollable(objJF, opts)
        end
       
        % Stabilizability function
        function stabilizabilityFlag = isstabilizable(objJF, opts)
        end
    end

    methods (Access = protected)
        function propgrp = getPropertyGroups(~)
            proplist = {'Ac', 'Bwc', 'Ad', 'Bwd', 'Czc', 'Dzc_wc', 'Czd', 'Dzd_wd'};
            propgrp = matlab.mixin.util.PropertyGroup(proplist);
        end
    end
end