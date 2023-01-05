function [flag, varargout] = wellConditionedController(K)
% varargout [nup, ntz]
arguments
    K           {mustBeDynamicalSystem(K)}
end

nargoutchk(0, 2);

z = tzero(K);
p = pole(K);

if K.Ts ~= 0
    nunstabpoles = sum(abs(p)>=1);
    nnonminzeros = sum(abs(z)>=1);
else
    nunstabpoles = sum(real(p)>=0);
    nnonminzeros = sum(real(z)>=0);
end

if nunstabpoles+nnonminzeros > 0
    flag = false;
else
    flag = true;
end

if nargout == 2
    flag = nunstabpoles;
    varargout(1) = {nnonminzeros};
end

end