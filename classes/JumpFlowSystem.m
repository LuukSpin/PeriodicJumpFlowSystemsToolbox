classdef JumpFlowSystem < handle & matlab.mixin.CustomDisplay
    %UNTITLED11 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
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
        nx
        nwc
        nzc
        nwd
        nzd
    end
    
    properties (GetAccess = 'public', SetAccess = ?OpenLoopJumpFlowSystem)
        Loop = 'Closed'
    end
    
    % Constructor
    methods
        function obj = JumpFlowSystem(Ac, Bwc, Ad, Bwd, Czc, Dzc_wc, Czd, Dzd_wd)
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
    end
    
    methods (Access = protected)
        function propgrp = getPropertyGroups(~)
            proplist = {'Loop', 'Ac', 'Bwc', 'Ad', 'Bwd', 'Czc', 'Dzc_wc', 'Czd', 'Dzd_wd'};
            propgrp = matlab.mixin.util.PropertyGroup(proplist);
        end
    end
end

