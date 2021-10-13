function stabilityFlag = JFStability(CLJFobj, h)
%JFSTABILITY checks the stability of a closed-loop jump-flow system
%
%   

%Determine the fundamental monodromy matrix P=Phi = Ad*exp(Ac(h)
Phi = expm(CLJFobj.Ac*h)*CLJFobj.Ad;
Phi_eig = eig(Phi);
stabilityFlag = all(abs(Phi_eig)<1);

end

