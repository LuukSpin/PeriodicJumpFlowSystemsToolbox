function normValue = analysis(OLSDSystem, performanceIndicator, opts)

arguments
    OLSDSystem              (1,1) OpenLoopSampledDataSystem
    performanceIndicator    (1,1) string
    opts                    (1,1) SDopts
end

% Check stability
if ~OLSDSystem.isstable(h)
    warning('The system is not stable and hence does not have a finite norm of any kind.');
    normValue = nan;
    return
end

Ac = OLSDSystem.Ac;
Bwc = [OLSDSystem.Bwc, OLSDSystem.Buc];
Ad = OLSDSystem.Ad;
Bwd = [OLSDSystem.Bwd, OLSDSystem.Bud];
Czc = OLSDSystem.Czc;
Dzc_wc = [OLSDSystem.Dzc_wc, OLSDSystem.Dzc_uc];
Czd = [OLSDSystem.Czd; OLSDSystem.Cy];
Dzd_wd = [OLSDSystem.Dzd_wd, OLSDSystem.Dzd_ud; OLSDSystem.Dy_wd, OLSDSystem.Dy_ud];

% Create JumpFlowSystem object from OpenLoopSampledDataSystem object
OLJFSystem = JumpFlowSystem(Ac, Bwc, Ad, Bwd, Czc, Dzc_wc, Czd, Dzd_wd);

% Use analysis function defined for JumpFlowSystem objects
normValue = analysis(OLJFSystem, performanceIndicator, opts);
end