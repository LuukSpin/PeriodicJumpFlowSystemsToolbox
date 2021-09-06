classdef OpenLoopJumpFlowSystem < JumpFlowSystem
    %UNTITLED15 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %Controller output to state matrices
        Bud
        
        %Controller output to performance channel feed-through terms
        Dzd_u
        
        %Controller input matrices
        Cy
        Dy_wd
        Dy_u
    end
    
    methods
        function obj = OpenLoopJumpFlowSystem()
            %UNTITLED15 Construct an instance of this class
            %   Detailed explanation goes here
            obj=obj@JumpFlowSystem();
            obj.Loop = 'Open';
        end
    end
    methods (Access = protected)
        function propgrp = getPropertyGroups(~)
            proplist = {'Loop', 'Ac', 'Bwc', 'Ad', 'Bwd', 'Bud', 'Czc', 'Dzc_wc', 'Czd', 'Dzd_wd', 'Dzd_u','Cy','Dy_wd', 'Dy_u'};
            propgrp = matlab.mixin.util.PropertyGroup(proplist);
        end
    end
end

