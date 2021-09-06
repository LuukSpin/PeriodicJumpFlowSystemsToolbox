classdef OpenLoopSampledDataSystem < SampledDataSystem
    %UNTITLED15 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %Controller output to state matrices
        Buc
        Bud
        
        %Controller output to performance channel feed-through terms
        Dzc_u
        Dzd_u
        
        %Controller input matrices
        Cy
        Dy_wd
        Dy_u
    end
    
    methods
        function obj = OpenLoopSampledDataSystem()
            %UNTITLED15 Construct an instance of this class
            %   Detailed explanation goes here
            obj=obj@SampledDataSystem();
            obj.Loop = 'Open';
        end
    end
    methods (Access = protected)
        function propgrp = getPropertyGroups(~)
            proplist = {'Weighting','Loop', 'Ac', 'Bwc', 'Buc', 'Ad', 'Bwd', 'Bud', 'Czc', 'Dzc_wc', 'Dzc_u', 'Czd', 'Dzd_wd', 'Dzd_u','Cy','Dy_wd', 'Dy_u'};
            propgrp = matlab.mixin.util.PropertyGroup(proplist);
        end
    end
end

