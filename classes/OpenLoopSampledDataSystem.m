classdef OpenLoopSampledDataSystem < SampledDataSystem
    %OPENLOOPSAMPLEDDATASYSTEM specifies an open-loop sampled-data system
    %   Detailed explanation goes here

    properties
        %Controller output to state matrices
        Buc     double {mustBeFinite(Buc)}
        Bud     double {mustBeFinite(Bud)}

        %Controller output to performance channels feed-through terms
        Dzc_u   double {mustBeFinite(Dzc_u)}
        Dzd_u   double {mustBeFinite(Dzd_u)}

        %Controller input matrices
        Cy      double {mustBeFinite(Cy)}
        Dy_wd   double {mustBeFinite(Dy_wd)}
        Dy_u    double {mustBeFinite(Dy_u)}
    end

    properties (Dependent = true, SetAccess = private, GetAccess = 'public')
        nu
        ny
    end

    methods
        function obj = OpenLoopSampledDataSystem(Ac, Bwc, Buc, Ad, Bwd, Bud, Czc, Dzc_wc, Dzc_u, Czd, Dzd_wd, Dzd_u, Cy, Dy_wd, Dy_u)
            %OpenLoopSampledDataSystem Construct an instance of this class
            %   OpenLoopSampledDataSystem(Ac, Bwc, Buc, Ad, Bwd, Bud, Czc,
            %   Dzc_wc, Dzc_u, Czd, Dzd_Wd, Dzd_u, Cy, Cy_wd, Cy_u)
            %   construct an open-loop sampled-data system with the
            %   following state-space realization:
            %  \dot{x} = Ac*x  + Bwc*w_c    + Buc*u
            %   x^+    = Ad*x  + Bwd*w_d    + Bud*\hat{u}
            %   z_c    = Czc*x + Dzc_wc*w_c + Dzc_u*u
            %   z_d    = Czd*x + Dzd_wd*w_d + Dzd_u*\hat{u}
            %   y      = Cy*x  + Dy_wd*w_d  + Dy_u*\hat{u}

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
                Dzc_u = zeros(size(Czc, 1), size(Buc, 2))
                Czd = zeros(0, size(Ac, 1))
                Dzd_wd = zeros(size(Czd, 1), size(Bwd, 2))
                Dzd_u = zeros(size(Czd, 1), size(Buc, 2))
                Cy = zeros(0, size(Ac, 1))
                Dy_wd = zeros(size(Cy, 1), size(Bwd, 2))
                Dy_u = zeros(size(Cy, 1), size(Buc, 2))
            end

            obj=obj@SampledDataSystem(Ac, Bwc, Ad, Bwd, Czc, Dzc_wc, Czd, Dzd_wd);

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

            % Check controller output to continuous-time performance channels feed-through matrix
            nzc = size(Czc, 1);

            if nzc ~= size(Dzc_u, 1)
                error('The number of rows of "Dzc_u" does not match the amount of continuous-time performance channels');
            end

            nu = size(Buc, 2);

            if nu ~= size(Dzc_u, 2)
                error('The number of columns of "Dzc_u" does not match the controller output dimension');
            end

            % Check controller output to discrete-time performance channels feed-through matrix
            nzd = size(Czd, 1);

            if nzd ~= size(Dzd_u, 1)
                error('The number of rows of "Dzd_u" does not match the amount of discrete-time performance channels');
            end

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

            % Controller output to state matrices
            obj.Buc = Buc;
            obj.Bud = Bud;

            % Controller output to performance channels feed-through matrices
            obj.Dzc_u = Dzc_u;
            obj.Dzd_u = Dzd_u;

            % Controller input matrices
            obj.Cy = Cy;
            obj.Dy_wd = Dy_wd;
            obj.Dy_u = Dy_u;

            % Set Loop to "Open"
            obj.Loop = 'Open';
        end

        % When OpenLoopSampledDataSystem.nu is called it is calculated based on the
        % Buc property
        function nu = get.nu(OpenLoopSampledDataSystem)
            nu = size(OpenLoopSampledDataSystem.Buc, 2);
        end

        % When OpenLoopSampledDataSystem.ny is called it is calculated based on the
        % Cy property
        function ny = get.ny(OpenLoopSampledDataSystem)
            ny = size(OpenLoopSampledDataSystem.Cy, 1);
        end

    end
    methods (Access = protected)
        function propgrp = getPropertyGroups(~)
            proplist = {'Loop', 'Ac', 'Bwc', 'Buc', 'Ad', 'Bwd', 'Bud', 'Czc', 'Dzc_wc', 'Dzc_u', 'Czd', 'Dzd_wd', 'Dzd_u','Cy','Dy_wd', 'Dy_u'};
            propgrp = matlab.mixin.util.PropertyGroup(proplist);
        end
    end
end