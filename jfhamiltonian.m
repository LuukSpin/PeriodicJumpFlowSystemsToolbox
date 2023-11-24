function [H, M2] = jfhamiltonian(sys, opts)

arguments
    sys         {mustBeA(sys, ["JumpFlowSystem", "OpenLoopSampledDataSystem"])}
    opts        jfopt
end

% Check all specified system norms such as Hinf, H2, H2g, L1
switch lower(opts.performanceString)
    case {'hinf', 'l2', 'h-inf'}
        Qc = opts.performanceValue*eye(sys.nwc);
        Sc = zeros(sys.nwc, sys.nzc);
        Rc = -opts.performanceValue\eye(sys.nzc);
    case {'h2'}
        error('H2-norm is not yet implemented');
    case {'h2g'}
        error('genH2-norm is not yet implemented');
    case {'l1'}
        error('L1-norm is not yet implemented');
    case {'passivity', 'passive', 'pass'}
        Qc = zeros(sys.nwc);
        Sc = eye(sys.nwc, sys.nzc)/2;
        Rc = zeros(sys.nzc);
    case {'qrs', 'quad', 'quadratic'}
        Qc = opts.performanceValue.Qc;
        if ~all(Qc==Qc', 'all')
            error('Continuos performance matrix Qc must be symmetric');
        end
        Sc = opts.performanceValue.Sc;
        Rc = opts.performanceValue.Rc;
        if ~all(Rc==Rc', 'all')
            error('Continuos performance matrix Rc must be symmetric');
        elseif any(eig(Rc)>=0)
            if any(eig(Rc) > 0)
                error('Discrete performance matrix Rc must be negative semi-definite');
            elseif any(eig(Rc) == 0)
                if ~all(eig(Rc)==0)
                    error('Discrete performance matrix Rc must be negative definite or the zero matrix');
                end
            end
        end
end

if isa(sys, 'OpenLoopSampledDataSystem')
    [Ac, Bc, Cc, Dc] = ClosedLoopFlowMatrices(sys, opts);
else
    Ac = sys.Ac;
    Bc = sys.Bwc;
    Cc = sys.Czc;
    Dc = sys.Dzc_wc;
end

M1 = Cc'*Rc*Cc;
M2 = Qc+Sc*Dc + Dc'*Sc'+Dc'*Rc*Dc;
M3 = Cc'*(Sc'+Rc*Dc);

H11 = Ac-Bc/M2*M3';
H12 = Bc/M2*Bc';
H21 = M1-M3/M2*M3';

H = [H11, H12; H21, -H11'];

end