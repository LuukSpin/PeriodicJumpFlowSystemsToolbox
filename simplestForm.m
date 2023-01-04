function simplestFormString = simplestForm(obj)

arguments
    obj   (1,1) {mustBeA(obj, ["JumpFlowSystem", "OpenLoopSampledDataSystem", "OpenLoopJumpFlowSystem"])}
end

if strcmpi(class(obj), 'OpenLoopJumpFlowSystem')

    nyf = obj.nyf;
    nyj = obj.nyj;
    nuf = obj.nuf;
    nuj = obj.nuj;
    nwc = obj.nwc;
    nwd = obj.nwd;
    nzc = obj.nzc;
    nzd = obj.nzd;

    if nyf == 0
        if nuf == nuj
            simplestFormString = 'sd';
        elseif nuf == 0 && nzc == 0 && nwc == 0
            simplestFormString = 'dt';
        end
    elseif nyj == 0 && nuj == 0 && nzd == 0 && nwd == 0 && all(obj.Ad == eye(size(obj.Ad)), 'all')
        simplestFormString = 'ct';
    else
        simplestFormString = 'jf';
    end
elseif strcmpi(class(obj), 'OpenLoopSampledDataSystem')
    
    ny = obj.ny; % ny = nyf, nyj = 0
    nwc = obj.nwc;
    nwd = obj.nwd;
    nzc = obj.nzc;
    nzd = obj.nzd;

    if isempty(obj)
        simplestFormString = 'sd';
    else
        obj = obj.applyReconstructor();
        if nwc == 0 && nzc == 0
            simplestFormString = 'dt';
        elseif ny == 0 && nwd == 0 && nzd == 0 && all(obj.Ad == eye(size(obj.Ad)), 'all') && all(obj.Bud == zeros(size(obj.Bud)), 'all')
            simplestFormString = 'ct';
        else
            simplestFormString = 'sd';
        end
    end
else % Class is JumpFlowSystem
    nwc = obj.nwc;
    nwd = obj.nwd;
    nzc = obj.nzc;
    nzd = obj.nzd;

    if nwc == 0 && nzc == 0
        simplestFormString = 'dt';
    elseif nwd == 0 && nzd == 0 && all(obj.Ad == eye(size(obj.Ad)), 'all')
        simplestFormString = 'ct';
    else
        simplestFormString = 'jf';
    end
end