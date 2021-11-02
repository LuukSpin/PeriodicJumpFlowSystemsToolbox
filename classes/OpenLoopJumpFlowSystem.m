classdef OpenLoopJumpFlowSystem < JumpFlowSystem
    %UNTITLED15 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        %Controller output to state matrices
        Bud     double {mustBeFinite(Bud)}

        %Controller output to performance channel feed-through terms
        Dzd_u   double {mustBeFinite(Dzd_u)}

        %Controller input matrices
        Cy      double {mustBeFinite(Cy)}
        Dy_wd   double {mustBeFinite(Dy_wd)}
        Dy_u    double {mustBeFinite(Dy_u)}
    end

    properties (Dependent = true, SetAccess = private, GetAccess = 'public')
        nu      (1,1) double
        ny      (1,1) double
    end

    methods
        function obj = OpenLoopJumpFlowSystem(Ac, Bwc, Ad, Bwd, Bud, Czc, Dzc_wc, Czd, Dzd_wd, Dzd_u, Cy, Dy_wd, Dy_u)
            %OpenLoopJumpFlowSystem Construct an instance of this class
            %   OpenLoopJumpFlowSystem(Ac, Bwc, Ad, Bwd, Bud, Czc, Dzc_wc,
            %   Czd, Dzd_Wd, Dzd_u, Cy, Cy_wd, Cy_u) construct an open-loop
            %   jump-flow system. A jump-flow system is sampled-data where
            %   discrete-time the controller output is connected to the
            %   continuous-time plant through a Zero-Order-Hold. This
            %   system has the following state-space realization:
            %  \dot{x} = Ac*x  + Bwc*w_c
            %   x^+    = Ad*x  + Bwd*w_d    + Bud*\hat{u}
            %   z_c    = Czc*x + Dzc_wc*w_c
            %   z_d    = Czd*x + Dzd_wd*w_d + Dzd_u*\hat{u}
            %   y      = Cy*x  + Dy_wd*w_d  + Dy_u*\hat{u}

            % Here the arguments is setup such that the input arguments are
            % variable. That means that this constructor accepts 0 up to 15
            % input arguments. However, no more than 15 input arguments is
            % allowed.
            arguments
                Ac = []
                Bwc = zeros(size(Ac, 1), 0)
                Ad = zeros(size(Ac))
                Bwd = zeros(size(Ac, 1), 0)
                Bud = zeros(size(Ac, 1), 0)
                Czc = zeros(0, size(Ac, 1))
                Dzc_wc = zeros(size(Czc, 1), size(Bwc, 2))
                Czd = zeros(0, size(Ac, 1))
                Dzd_wd = zeros(size(Czd, 1), size(Bwd, 2))
                Dzd_u = zeros(size(Czd, 1), size(Bud, 2))
                Cy = zeros(0, size(Ac, 1))
                Dy_wd = zeros(size(Cy, 1), size(Bwd, 2))
                Dy_u = zeros(size(Cy, 1), size(Bud, 2))
            end

            obj=obj@JumpFlowSystem(Ac, Bwc, Ad, Bwd, Czc, Dzc_wc, Czd, Dzd_wd);

            % Make sure the matrices have the appropriate sizes (the
            % matrices used above are checked in the constructor of
            % SampledDataSystem

            % Check controller output to state matrices
            nx = size(Ac, 1);

            if nx ~= size(Bud, 1)
                error('The number of rows of "Bud" does not match the state dimension');
            end

            % Check controller output to discrete-time performance channels feed-through matrix
            nzd = size(Czd, 1);

            if nzd ~= size(Dzd_u, 1)
                error('The number of rows of "Dzd_u" does not match the amount of discrete-time performance channels');
            end

            nu = size(Bud, 2);

            if nu ~= size(Dzd_u, 2)
                error('The number of columns of "Dzd_u" does not match the controller output dimension');
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

            if ny ~= size(Dy_u, 1)
                error('The number of rows of "Dy_u" does not match the controller input dimension');
            end

            if nu ~= size(Dy_u, 2)
                error('The number of columns of "Dy_u" does not match the controller output dimension');
            end

            % Controller output to state matrix
            obj.Bud = Bud;

            % Controller output to performance channels feed-through matrix
            obj.Dzd_u = Dzd_u;

            % Controller input matrices
            obj.Cy = Cy;
            obj.Dy_wd = Dy_wd;
            obj.Dy_u = Dy_u;

            % Set Loop to "Open"
            obj.Loop = 'Open';
        end

        % When OpenLoopJumpFlowSystem.nu is called it is calculated based on the
        % Bud property
        function nu = get.nu(OpenLoopJumpFlowSystem)
            nu = size(OpenLoopJumpFlowSystem.Bud, 2);
        end

        % When OpenLoopJumpFlowSystem.ny is called it is calculated based on the
        % Cy property
        function ny = get.ny(OpenLoopJumpFlowSystem)
            ny = size(OpenLoopJumpFlowSystem.Cy, 1);
        end

        % Determine closed-loop matrices based on open-loop jump-flow
        % system
        function [Ac, Bwc, Czc, Dzc_wc] = ClosedLoopFlowMatrices(OLJFSystem, nc)
            arguments
                OLJFSystem      (1,1) OpenLoopJumpFlowSystem
                nc              (1,1) double = OLJFSystem.nx
            end

            %dimensions
            nwc = OLJFSystem.nwc;
            nzc = OLJFSystem.nzc;

            Ac = blkdiag(OLJFSystem.Ac, zeros(nc));
            Bwc = [OLJFSystem.Bwc; zeros(nc, nwc)];
            Czc = [OLJFSystem.Czc, zeros(nzc, nc)];
            Dzc_wc = OLJFSystem.Dzc_wc;

        end

        function objJF = lft(OLJFSystem, DiscreteController)
            arguments
                OLJFSystem          (1,1) OpenLoopJumpFlowSystem
                DiscreteController  {mustBeNumericOrListedType(DiscreteController, "ss", "tf")}
            end

            Controller = ss(DiscreteController);

            if Controller.Ts == 0
                error('The controller has to be a discrete-time controller');
            end

            % Sampling time
            h = Controller.Ts;

            % Controller matrices
            Acontroller = Controller.A;
            Bcontroller = Controller.B;
            Ccontroller = Controller.C;
            Dcontroller = Controller.D;
            controllerMat = [Acontroller, Bcontroller; Ccontroller, Dcontroller];

            % Dimensions
            nx = OLJFSystem.nx;
            nc = size(Acontroller, 1);
            nwd = OLJFSystem.nwd;
            nzd = OLJFSystem.nzd;

            % Determine all closed-loop flow matrices
            [Ac, Bwc, Czc, Dzc_wc] = OLJFSystem.ClosedLoopFlowMatrices(nc);

            % Jump matrices
            AdAdd = blkdiag(OLJFSystem.Ad, zeros(nc));
            AdLeft = [zeros(nx, nc), OLJFSystem.Bud; eye(nc), zeros(nc, OLJFSystem.nu)];
            AdRight = [zeros(nc, nx), eye(nc); OLJFSystem.Cy, zeros(OLJFSystem.ny, nc)];
            Ad = AdAdd+AdLeft*controllerMat*AdRight;
            
            BwdAdd = [OLJFSystem.Bwd; zeros(nc, nwd)];
            BwdLeft = AdLeft;
            BwdRight = [zeros(nc, nwd); OLJFSystem.Dy_wd];
            Bwd = BwdAdd+BwdLeft*controllerMat*BwdRight;

            % Discrete-time performance channel matrices
            CzdAdd = [OLJFSystem.Czd, zeros(nzd, nc)];
            CzdLeft = [zeros(nzd, nc), OLJFSystem.Dzd_u];
            CzdRight = AdRight;
            Czd = CzdAdd+CzdLeft*controllerMat*CzdRight;

            Dzd_wdAdd = OLJFSystem.Dzd_wd;
            Dzd_wdLeft = CzdLeft;
            Dzd_wdRight = BwdRight;
            Dzd_wd = Dzd_wdAdd+Dzd_wdLeft*controllerMat*Dzd_wdRight;

            % Initiate closed-loop jump-flow system
            objJF = JumpFlowSystem(Ac, Bwc, Ad, Bwd, Czc, Dzc_wc, Czd, Dzd_wd);

            if ~objJF.isstable(h)
                warning('Closing the loop with the specified controller does not results in a stable closed-loop system!');
            end

        end
    end
    methods (Access = protected)
        function propgrp = getPropertyGroups(~)
            proplist = {'Loop', 'Ac', 'Bwc', 'Ad', 'Bwd', 'Bud', 'Czc', 'Dzc_wc', 'Czd', 'Dzd_wd', 'Dzd_u','Cy','Dy_wd', 'Dy_u'};
            propgrp = matlab.mixin.util.PropertyGroup(proplist);
        end
    end
end

