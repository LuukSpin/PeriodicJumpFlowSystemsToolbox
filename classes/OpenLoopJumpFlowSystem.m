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
            %CLOSEDLOOPFLOWMATRICES calculates continuous-time flow matrices (Ac, Bwc,
            %Czc, Dzc_wc) fromt the open-loop jump-flow system.
            %
            %   [Ac, Bc, Cc, Dc] = CLOSEDLOOPFLOWMATRICES(OLJFSystem, nc) returns the
            %   closed-loop flow matrices based on an open-loop jump-flow system. Here
            %   nc denotes the dimension of the controller.
            %
            %   The open-loop jump-flow system has the following state-space
            %   realization.
            %
            %  \dot{x} = Ac*x  + Bwc*w_c
            %   x^+    = Ad*x  + Bwd*w_d    + Bud*\hat{u}
            %   z_c    = Czc*x + Dzc_wc*w_c
            %   z_d    = Czd*x + Dzd_wd*w_d + Dzd_u*\hat{u}
            %   y      = Cy*x  + Dy_wd*w_d
            %
            %   The closed-loop jump flow system has the following state-space
            %   realization
            %
            %  \xi   = Ac*\xi  + Bc*w_c
            %  \xi^+ = Ad*\xi  + Bd*w_d
            %   z_c  = Cc*\xi + Dc*w_c
            %   z_d  = Cd*\zi + Dd*w_d

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

        function objJF = lft(OLJFSystem1, OLJFSystem2)
            %LFT is used to interconnect systems
            %   LFT(sys1, sys2) produces a closed-loop jump-flow system.
            %   sys1 has to be an OpenLoopJumpFlowSystem while sys2 may be
            %   a discrete controller. The interconnection looks as
            %   follows:
            %

            arguments
                OLJFSystem1         (1,1) OpenLoopJumpFlowSystem
                OLJFSystem2         {mustBeNumericOrListedType(OLJFSystem2, "ss", "tf", "OpenLoopJumpFlowSystem")}
            end

            % If the second input is a controller than convert it to a
            % OLJFSystem
            if ~isa(OLJFSystem2, 'OpenLoopJumpFlowSystem')
                OLJFSystem2 = ss(OLJFSystem2);
                h = OLJFSystem2.Ts;
                OLJFSystem2 = makeJFFromDiscreteController(OLJFSystem2);
            end

            % Make sure that the interconnection has a compatible
            % connection
            y1 = size(OLJFSystem1.Cy, 1);
            u1 = size(OLJFSystem1.Bud, 2);
            y2 = size(OLJFSystem2.Cy, 1);
            u2 = size(OLJFSystem2.Bud, 2);

            if (y1 ~= u2) || (u1 ~= y2)
                error('The interconnection is not possible because the dimensions of the connection does not match');
            end

            Dyu1 = OLJFSystem1.Dy_u;
            Dyu2 = OLJFSystem2.Dy_u;

            % Interconnection must be wellposed, can be checked by
            % feed-through terms
            wellPosednessMatrix = [eye(y2), -Dyu2; -Dyu1, eye(y1)];
            if det(wellPosednessMatrix) == 0
                error('This interconnection is not well-posed, and hence the interconnection will not result in a non-causal system. The interconnection is aborted.');
            end

            % Flow and continuous-time performance channels matrices
            Ac = blkdiag(OLJFSystem1.Ac, OLJFSystem2.Ac);
            Bwc = blkdiag(OLJFSystem1.Bwc, OLJFSystem2.Bwc);
            Czc = blkdiag(OLJFSystem1.Czc, OLJFSystem2.Czc);
            Dzc_wc= blkdiag(OLJFSystem1.Dzc_wc, OLJFSystem2.Dzc_wc);

            % Define matrices used in the partitioning that follows
            R12 = eye(y1)-Dyu1*Dyu2;
            R21 = eye(y2)-Dyu2*Dyu1;
            Ad1 = OLJFSystem1.Ad;
            Ad2 = OLJFSystem2.Ad;
            Bu1 = OLJFSystem1.Bud;
            Bu2 = OLJFSystem2.Bud;
            Cy1 = OLJFSystem1.Cy;
            Cy2 = OLJFSystem2.Cy;
            Bd1 = OLJFSystem1.Bwd;
            Bd2 = OLJFSystem2.Bwd;
            Dyd1 = OLJFSystem1.Dy_wd;
            Dyd2 = OLJFSystem2.Dy_wd;
            Cd1 = OLJFSystem1.Czd;
            Cd2 = OLJFSystem2.Czd;
            Ddu1 = OLJFSystem1.Dzd_u;
            Ddu2 = OLJFSystem2.Dzd_u;
            Ddd1 = OLJFSystem1.Dzd_wd;
            Ddd2 = OLJFSystem2.Dzd_wd;

            % Jump and discrete-time performance channels matrices
            Ad = [Ad1 + Bu1/R21*Dyu2*Cy1, Bu1/R21*Cy2; Bu2/R12*Cy1, Ad2 + Bu2/R12*Dyu1*Cy2];
            Bwd = [Bd1 + Bu1/R21*Dyu2*Dyd1, Bu1/R21*Dyd2; Bu2/R12*Dyd1, Bd2 + Bu2/R12*Dyu1*Dyd2];
            Czd = [Cd1 + Ddu1/R21*Dyu2*Dyd1, Ddu1/R21*Dyd2; Ddu2/R12*Dyd1, Cd2 + Ddu2/R12*Dyu1*Dyd2];
            Dzd_wd = [Ddd1 + Ddu1/R21*Dyu2*Dyd1, Ddu1/R21*Dyd2; Ddu2/R12*Dyd1, Ddd2 + Ddu2/R12*Dyu1*Dyd2];

            objJF = JumpFlowSystem(Ac, Bwc, Ad, Bwd, Czc, Dzc_wc, Czd, Dzd_wd);

            % Check stability if second input is a controller
            if ~isa(OLJFSystem2, 'OpenLoopJumpFlowSystem')
                if ~objJF.isstable(h)
                    warning('The closed-loop interconnection of the two systems is not stable!');
                end
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

