classdef OpenLoopSampledDataSystem < JumpFlowSystem
    %OpenLoopSampledDataSystem Construct an instance of this class
    %   OpenLoopSampledDataSystem(Ac, Bwc, Buc, Ad, Bwd, Bud, Czc,
    %   Dzc_wc, Dzc_u, Czd, Dzd_Wd, Dzd_u, Cy, Cy_wd, Cy_u)
    %   construct an open-loop sampled-data system with the
    %   following state-space realization:
    %  \dot{x} = Ac*x  + Bwc*w_c    + Buc*u
    %   x^+    = Ad*x  + Bwd*w_d    + Bud*\hat{u}
    %   z_c    = Czc*x + Dzc_wc*w_c + Dzc_uc*u
    %   z_d    = Czd*x + Dzd_wd*w_d + Dzd_ud*\hat{u}
    %   y      = Cy*x  + Dy_wd*w_d  + Dy_ud*\hat{u}

    properties
        %Controller output to state matrices
        Buc     double {mustBeFinite(Buc)}
        Bud     double {mustBeFinite(Bud)}

        %Controller output to performance channels feed-through terms
        Dzc_uc  double {mustBeFinite(Dzc_uc)}
        Dzd_ud  double {mustBeFinite(Dzd_ud)}

        %Controller input matrices
        Cy      double {mustBeFinite(Cy)}
        Dy_wd   double {mustBeFinite(Dy_wd)}
        Dy_ud    double {mustBeFinite(Dy_ud)}
    end

    properties (Dependent = true, SetAccess = private, GetAccess = 'public')
        nu      (1,1) double
        ny      (1,1) double
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
        function SDSystem = uplus(objSD)
            arguments
                objSD (1,1) OpenLoopSampledDataSystem
            end

            SDSystem = objSD;
        end

        % Override the uminus operator for OpenLoopSampledDataSystem class object
        function SDSystem = uminus(objSD)
            arguments
                objSD (1,1) OpenLoopSampledDataSystem
            end

            Ac = objSD.Ac;
            Bwc = objSD.Bwc;
            Ad = objSD.Ad;
            Bwd = objSD.Bwd;
            Bud = objSD.Bud;
            Czc = -objSD.Czc;
            Dzc_wc = -objSD.Dzc_wc;
            Czd = -objSD.Czd;
            Dzd_wd = -objSD.Dzd_wd;
            Dzd_u = -objSD.Dzd_uc;
            Cy = -objSD.Cy;
            Dy_wd = -objSD.Dy_wd;
            Dy_u = -objSD.Dy_u;

            SDSystem = OpenLoopSampledDataSystem(Ac, Bwc, Ad, Bwd, Bud, Czc, Dzc_wc, Czd, Dzd_wd, Dzd_u, Cy, Dy_wd, Dy_u);
        end

        % Override the plus operator for OpenLoopSampledDataSystem class object
        function SDSystem = plus(objSD1, objSD2)
            arguments
                objSD1 (1,1) OpenLoopSampledDataSystem
                objSD2 (1,1) OpenLoopSampledDataSystem
            end

            Ac = blkdiag(objSD1.Ac, objSD2.Ac);
            Bwc = [objSD1.Bwc; objSD2.Bwc];
            Ad = blkdiag(objSD1.Ad, objSD2.Ad);
            Bwd = [objSD1.Bwd; objSD2.Bwd];
            Bud = [objSD1.Bud; objSD2.Bud];
            Czc = [objSD1.Czc, objSD2.Czc];
            Dzc_wc = objSD1.Dzc_wc+objSD2.Dzc_wc;
            Czd = [objSD1.Czd, objSD2.Czd];
            Dzd_wd = objSD1.Dzd_wd+objSD2.Dzd_wd;
            Dzd_u = objSD1.Dzd_uc+objSD2.Dzd_u;
            Cy = [objSD1.Cy, objSD2.Cy];
            Dy_wd = objSD1.Dy_wd+objSD2.Dy_wd;
            Dy_u = objSD1.Dy_u+objSD2.Dy_u;

            SDSystem = OpenLoopSampledDataSystem(Ac, Bwc, Ad, Bwd, Bud, Czc, Dzc_wc, Czd, Dzd_wd, Dzd_u, Cy, Dy_wd, Dy_u);
        end

        % Override the minus operator for OpenLoopSampledDataSystem class object
        function SDSystem = minus(objSD1, objSD2)
            arguments
                objSD1 (1,1) OpenLoopSampledDataSystem
                objSD2 (1,1) OpenLoopSampledDataSystem
            end

            SDSystem = objSD1 + (-objSD2);

        end
        
        % Override the mtimes operator for OpenLoopSampledDataSystem class object
        function SDSystem = mtimes(obj1, obj2)
        % MTIMES specifies the multiplication operator * for sampled-data
        % systems
        %   Multiplication of sampled-data systems specifies a series
        %   connection of system that looks like the following block
        %   diagram:
        %   
        %   
            arguments
                obj1 (1,1) OpenLoopSampledDataSystem
                obj2 (1,1) OpenLoopSampledDataSystem
            end

            nx1 = obj2.nx;
            nx2 = obj1.nx;
            nzc1 = obj2.nzc;
            nzd1 = obj2.nzd;
            ny1 = obj2.ny;
            nwc2 = obj1.nwc;
            nwd2 = obj1.nwd;
            nu2 = obj1.nu;

            if nzc1 ~= nwc2
                error('The amount of continuous-time performance channels of the first system is not equal to the amount of continuous-time disturbance channels of the second system.');
            end

            if nzd1 ~= nwd2
                error('The amount of discrete-time performance channels of the first system is not equal to the amount of discrete-time disturbance channels of the second system.');
            end

            if ny1 ~= nu2
                error('The controller input dimension of the first system does not match the controller output dimension of the second system.');
            end

            Ac = [obj2.Ac, zeros(nx1, nx2); obj1.Bwc*obj2.Czc, obj1.Ac];
            Bwc = [obj2.Bwc; obj1.Bwc*obj2.Dzc_wc];
            Ad = [obj2.Ad, zeros(nx1, nx2); obj1.Bwd*obj2.Czd+obj1.Bud*obj2.Cy, obj1.Ad];
            Bwd = [obj2.Bwd; obj1.Bwd*obj2.Dzd_wd+obj1.Bud*obj2.Dy_wd];
            Bud = [obj2.Bud; obj1.Bwd*obj2.Dzd_u+obj1.Bud*obj2.Dy_u];

            Czc = [obj1.Dzc_wc*obj2.Czc, obj1.Czc];
            Dzc_wc = obj1.Dzc_wc*obj2.Dzc_wc;

            Czd = [obj1.Dzd_wd*obj2.Czd+obj1.Dzd_uc*obj2.Cy, obj1.Czd];
            Dzd_wd = obj1.Dzd_wd*obj2.Dzd_wd+obj1.Dzd_uc*obj2.Dy_wd;
            Dzd_u = obj1.Dzd_wd*obj2.Dzd_u+obj1.Dzd_uc*obj2.Dy_u;

            Cy = [obj1.Dy_wd*obj2.Czd+obj1.Dy_u*obj2.Cy, obj1.Cy];
            Dy_wd = obj1.Dy_wd*obj2.Dzd_wd+obj1.Dy_u*obj2.Dy_wd;
            Dy_u = obj1.Dy_wd*obj2.Dzd_u+obj1.Dy_u*obj2.Dy_u;

            SDSystem = OpenLoopSampledDataSystem(Ac, Bwc, Ad, Bwd, Bud, Czc, Dzc_wc, Czd, Dzd_wd, Dzd_u, Cy, Dy_wd, Dy_u);

        end

        % Override the horzcat operator for OpenLoopSampledDataSystem class object
        function SDSystem = horzcat(obj1, obj2)
            arguments
                obj1 (1,1) OpenLoopSampledDataSystem
                obj2 (1,1) OpenLoopSampledDataSystem
            end

            Ac = blkdiag(obj1.Ac, obj2.Ac);
            Bwc = blkdiag(obj1.Bwc, obj2.Bwc);
            Ad = blkdiag(obj1.Ad, obj2.Ad);
            Bwd = blkdiag(obj1.Bwd, obj2.Bwd);
            Bud = blkdiag(obj1.Bud, obj2.Bud);

            Czc = [obj1.Czc, obj2.Czc];
            Dzc_wc = [obj1.Dzc_wc, obj2.Dzc_wc];
            Czd = [obj1.Czd, obj2.Czd];
            Dzd_wd = [obj1.Dzd_wd, obj2.Dzd_wd];
            Dzd_u = [obj1.Dzd_uc, obj2.Dzd_u];
            Cy = [obj1.Cy, obj2.Cy];
            Dy_wd = [obj1.Dy_wd, obj2.Dy_wd];
            Dy_u = [obj1.Dy_u, obj2.Dy_u];

            SDSystem = OpenLoopSampledDataSystem(Ac, Bwc, Ad, Bwd, Bud, Czc, Dzc_wc, Czd, Dzd_wd, Dzd_u, Cy, Dy_wd, Dy_u);
        end

        % Override the vertcat operator for OpenLoopSampledDataSystem class object
        function SDSystem = vertcat(obj1, obj2)
            arguments
                obj1 (1,1) OpenLoopSampledDataSystem
                obj2 (1,1) OpenLoopSampledDataSystem
            end

            Ac = blkdiag(obj1.Ac, obj2.Ac);
            Bwc = [obj1.Bwc; obj2.Bwc];
            Ad = blkdiag(obj1.Ad, obj2.Ad);
            Bwd = [obj1.Bwd; obj2.Bwd];
            Bud = [obj1.Bud; obj2.Bud];

            Czc = blkdiag(obj1.Czc, obj2.Czc);
            Dzc_wc = [obj1.Dzc_wc; obj2.Dzc_wc];
            Czd = blkdiag(obj1.Czd, obj2.Czd);
            Dzd_wd = [obj1.Dzd_wd; obj2.Dzd_wd];
            Dzd_u = [obj1.Dzd_uc; obj2.Dzd_u];
            Cy = blkdiag(obj1.Cy, obj2.Cy);
            Dy_wd = [obj1.Dy_wd; obj2.Dy_wd];
            Dy_u = [obj1.Dy_u; obj2.Dy_u];

            SDSystem = OpenLoopSampledDataSystem(Ac, Bwc, Ad, Bwd, Bud, Czc, Dzc_wc, Czd, Dzd_wd, Dzd_u, Cy, Dy_wd, Dy_u);
        end
        
        %% Sampled-data specific methods
        % Calculate closed-loop jump-flow system based on the
        % interconnection
        function objJF = lft(OLSDSystem1, OLSDSystem2)
            %LFT is used to interconnect systems
            %   LFT(sys1, sys2) produces a closed-loop jump-flow system.
            %   sys1 has to be an OpenLoopJumpFlowSystem while sys2 may be
            %   a discrete controller. The interconnection looks as
            %   follows:
            %

            arguments
                OLSDSystem1         (1,1) OpenLoopSampledDataSystem
                OLSDSystem2         {mustBeNumericOrListedType(OLSDSystem2, "ss", "tf", "OpenLoopSampledDataSystem")}
            end

            % If the second input is a controller than convert it to a
            % OLJFSystem
            if ~isa(OLSDSystem2, 'OpenLoopSampledDataSystem')
                OLSDSystem2 = ss(OLSDSystem2);
                h = OLSDSystem2.Ts;
                OLSDSystem2 = makeJFFromDiscreteController(OLSDSystem2);
            end

            % Make sure that the interconnection has a compatible
            % connection
            y1 = OLSDSystem1.ny;
            u1 = OLSDSystem1.nu;
            y2 = OLSDSystem2.ny;
            u2 = OLSDSystem2.nu;

            if (y1 ~= u2) || (u1 ~= y2)
                error('The interconnection is not possible because the dimensions of the connection does not match');
            end

            Dyu1 = OLSDSystem1.Dy_u;
            Dyu2 = OLSDSystem2.Dy_u;

            % Interconnection must be wellposed, can be checked by
            % feed-through terms
            wellPosednessMatrix = [eye(y2), -Dyu2; -Dyu1, eye(y1)];
            if det(wellPosednessMatrix) == 0
                error('This interconnection is not well-posed, and hence the interconnection will result in a non-causal system. The interconnection is aborted.');
            end

            % Flow and continuous-time performance channels matrices
            Ac = blkdiag(OLSDSystem1.Ac, OLSDSystem2.Ac);
            Bwc = blkdiag(OLSDSystem1.Bwc, OLSDSystem2.Bwc);
            Czc = blkdiag(OLSDSystem1.Czc, OLSDSystem2.Czc);
            Dzc_wc= blkdiag(OLSDSystem1.Dzc_wc, OLSDSystem2.Dzc_wc);

            % Define matrices used in the partitioning that follows
            R12 = eye(y1)-Dyu1*Dyu2;
            R21 = eye(y2)-Dyu2*Dyu1;
            Ad1 = OLSDSystem1.Ad;
            Ad2 = OLSDSystem2.Ad;
            Bu1 = OLSDSystem1.Bud;
            Bu2 = OLSDSystem2.Bud;
            Cy1 = OLSDSystem1.Cy;
            Cy2 = OLSDSystem2.Cy;
            Bd1 = OLSDSystem1.Bwd;
            Bd2 = OLSDSystem2.Bwd;
            Dyd1 = OLSDSystem1.Dy_wd;
            Dyd2 = OLSDSystem2.Dy_wd;
            Cd1 = OLSDSystem1.Czd;
            Cd2 = OLSDSystem2.Czd;
            Ddu1 = OLSDSystem1.Dzd_uc;
            Ddu2 = OLSDSystem2.Dzd_u;
            Ddd1 = OLSDSystem1.Dzd_wd;
            Ddd2 = OLSDSystem2.Dzd_wd;

            % Jump and discrete-time performance channels matrices
            Ad = [Ad1 + Bu1/R21*Dyu2*Cy1, Bu1/R21*Cy2; Bu2/R12*Cy1, Ad2 + Bu2/R12*Dyu1*Cy2];
            Bwd = [Bd1 + Bu1/R21*Dyu2*Dyd1, Bu1/R21*Dyd2; Bu2/R12*Dyd1, Bd2 + Bu2/R12*Dyu1*Dyd2];
            Czd = [Cd1 + Ddu1/R21*Dyu2*Cy1, Ddu1/R21*Cy2; Ddu2/R12*Cy1, Cd2 + Ddu2/R12*Dyu1*Cy2];
            Dzd_wd = [Ddd1 + Ddu1/R21*Dyu2*Dyd1, Ddu1/R21*Dyd2; Ddu2/R12*Dyd1, Ddd2 + Ddu2/R12*Dyu1*Dyd2];

            objJF = JumpFlowSystem(Ac, Bwc, Ad, Bwd, Czc, Dzc_wc, Czd, Dzd_wd);

            % Check stability if second input is a controller
            if ~isa(OLSDSystem2, 'OpenLoopSampledDataSystem')
                if ~objJF.isstable(h)
                    warning('The closed-loop interconnection of the two systems is not stable!');
                end
            end

        end

        % Determine closed-loop flow matrices based an open-loop sampled-data
        % system
        function [Ac, Bwc, Czc, Dzc_wc] = ClosedLoopFlowMatrices(OLSDSystem, nc)
            %CLOSEDLOOPFLOWMATRICES calculates closed-loop continuous-time flow matrices (Ac, Bwc,
            %Czc, Dzc_wc) from an open-loop sampled-data system.
            %
            %   [Ac, Bc, Cc, Dc] = CLOSEDLOOPFLOWMATRICES(OLJFSystem, nc) returns the
            %   closed-loop flow matrices based on an open-loop sampled-data system. Here
            %   nc denotes the dimension of the controller.
            %
            %   The open-loop sampled-data system has the following state-space
            %   realization.
            %
            %  \dot{x} = Ac*x  + Bwc*w_c
            %   x^+    = Ad*x  + Bwd*w_d    + Bud*\hat{u}
            %   z_c    = Czc*x + Dzc_wc*w_c
            %   z_d    = Czd*x + Dzd_wd*w_d + Dzd_u*\hat{u}
            %   y      = Cy*x  + Dy_wd*w_d + Dy_u*\hat{u}
            %
            %   The closed-loop jump-flow system has the following state-space
            %   realization
            %
            %  \xi   = Ac*\xi  + Bc*w_c
            %  \xi^+ = Ad*\xi  + Bd*w_d
            %   z_c  = Cc*\xi + Dc*w_c
            %   z_d  = Cd*\zi + Dd*w_d

            arguments
                OLSDSystem      (1,1) OpenLoopSampledDataSystem
                nc              (1,1) double = OLSDSystem.nx
            end

            %dimensions
            nwc = OLSDSystem.nwc;
            nzc = OLSDSystem.nzc;

            Ac = blkdiag(OLSDSystem.Ac, zeros(nc));
            Bwc = [OLSDSystem.Bwc; zeros(nc, nwc)];
            Czc = [OLSDSystem.Czc, zeros(nzc, nc)];
            Dzc_wc = OLSDSystem.Dzc_wc;

        end

        % Perform synthesis for various system gains and norms
        function [Controller, synthesisNormValue, CLJFSystem] = synthesis(OLSDSystem, performanceIndicator, h, opts)

            arguments
                OLSDSystem              (1,1) OpenLoopSampledDataSystem
                performanceIndicator    (1,1) string
                h                       (1,1) double
                opts                    (1,1) SDopts = SDopts(h)
            end

            % Check if the jump-flow system is a generalized plant (check
            % if a stabilizing controller exists)
            %             if ~OLJFSystem.isgenplant()
            %                 error('This system is not a generalized plant and hence cannot be stabilized');
            %             end

            % Check all specified system norms such as Hinf, H2, H2g, L1
            switch performanceIndicator
                case {'Hinf', 'L2', 'H-inf', 'hinf', 'l2', 'h-inf'}
                    [Controller, synthesisNormValue, CLJFSystem] = JFHinfsyn(OLSDSystem, h, opts);
                case {'H2', 'h2'}
                    warning('The H2 norm has yet to be implemented in the sampled-data toolbox');
                    Controller = 0;
                    synthesisNormValue = nan;
                    CLJFSystem = JumpFlowSystem();
                case {'H2g', 'h2g'}
                    warning('The generalized H2 norm has yet to be implemented in the sampled-data toolbox');
                    Controller = 0;
                    synthesisNormValue = nan;
                    CLJFSystem = JumpFlowSystem();
                case {'L1', 'l1'}
                    warning('The L1 norm has yet to be implemented in the sampled-data toolbox');
                    Controller = 0;
                    synthesisNormValue = nan;
                    CLJFSystem = JumpFlowSystem();
                case {'QRS', 'Quad', 'Quadratic', 'qrs', 'quad', 'quadratic'}
                    warning('Controller synthesis based on quadratic dissipativty is not yet implemented in the sampled-data toolbox');
                    Controller = 0;
                    synthesisNormValue = nan;
                    CLJFSystem = JumpFlowSystem();
                otherwise
                    error('The chosen performance indicator string is not a valid choice as it does not represent a system norm or gain.');
            end
        end
    end
    methods (Access = protected)
        function propgrp = getPropertyGroups(~)
            proplist = {'Loop', 'Ac', 'Bwc', 'Buc', 'Ad', 'Bwd', 'Bud', 'Czc', 'Dzc_wc', 'Dzc_uc', 'Czd', 'Dzd_wd', 'Dzd_ud','Cy','Dy_wd', 'Dy_ud'};
            propgrp = matlab.mixin.util.PropertyGroup(proplist);
        end
    end
end