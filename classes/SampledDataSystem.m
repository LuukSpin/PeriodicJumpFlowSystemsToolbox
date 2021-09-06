classdef SampledDataSystem < handle & matlab.mixin.CustomDisplay
    %UNTITLED10 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %Flow matrices
        Ac  double {mustBeFinite}
        Bwc double {mustBeFinite(Bwc)}
        
        %Jump Matrices
        Ad
        Bwd
        
        %Continuous-time performance channel matrices
        Czc
        Dzc_wc
        
        %Discrete-time performance channel matrices
        Czd
        Dzd_wd
        
    end
    
    properties (GetAccess = 'public', SetAccess = ?OpenLoopSampledDataSystem)
        Loop = 'Closed'
    end
    
    methods
        function obj = SampledDataSystem()
            %UNTITLED10 Construct an instance of this class
            %   Detailed explanation goes here
        end
    end
    methods (Access = protected)
        function propgrp = getPropertyGroups(~)
            proplist = {'Loop', 'Ac', 'Bwc', 'Ad', 'Bwd', 'Czc', 'Dzc_wc', 'Czd', 'Dzd_wd'};
            propgrp = matlab.mixin.util.PropertyGroup(proplist);
        end 
    end
end
