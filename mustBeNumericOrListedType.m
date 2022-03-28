function mustBeNumericOrListedType(a,varargin)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
     if ~isnumeric(a) && ~ismember(class(a),cellstr(varargin))
        eid = 'Type:notValid';
        msg = 'Argument must be numeric or one of the prescribed types';
        throwAsCaller(MException(eid,msg))
    end
end