function stabilityFlag = JFStability(CLJFobj, h)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%Determine the fundamental monodromy matrix P=Phi = Ad*exp(Ac(h)
Phi = CLJFobj.Ad*expm(CLJFobj.Ac*h);
Phi_eig = eig(Phi);
stabilityFlag = all(abs(Phi_eig)<1);

end

