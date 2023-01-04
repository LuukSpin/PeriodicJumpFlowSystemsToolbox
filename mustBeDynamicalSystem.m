function mustBeDynamicalSystem(a)%,varargin)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
     if ~isa(a, 'InputOutputModel') && ~isa(a, 'JumpFlowSystem')% && ~ismember(class(a),cellstr(varargin))
        eid = 'Type:notValid';
        msg = 'Argument must be a dynamical system';
        throwAsCaller(MException(eid,msg))
    end
end