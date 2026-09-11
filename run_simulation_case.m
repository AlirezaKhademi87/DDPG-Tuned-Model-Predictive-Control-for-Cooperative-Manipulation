function result = run_simulation_case(useAdaptiveTuning)
%RUN_SIMULATION_CASE Run one fixed or adaptive cooperative-MPC simulation.

cfg = simulation_parameters(useAdaptiveTuning);
ref = build_reference_trajectory(cfg);
disturbance = generate_disturbance(cfg);
bounds = build_mpc_bounds(cfg);

[qL, okL] = ik3R(ref.PL(:,1), ref.phiL(1), ...
    cfg.baseL, cfg.L1, cfg.L2, cfg.L3);
[qR, okR] = ik3R(ref.PR(:,1), ref.phiR(1), ...
    cfg.baseR, cfg.L1, cfg.L2, cfg.L3);

if ~okL || ~okR
    error('Initial inverse-kinematics solution is infeasible.');
end

uLPrev = zeros(3,1);
uRPrev = zeros(3,1);
rhoPrev = cfg.rho0;
z0 = bounds.z0;

log.qL = nan(3,cfg.N);
log.qR = nan(3,cfg.N);
log.uL = nan(3,cfg.N);
log.uR = nan(3,cfg.N);
log.uL_real = nan(3,cfg.N);
log.uR_real = nan(3,cfg.N);
log.pL = nan(2,cfg.N);
log.pR = nan(2,cfg.N);
log.fnL = nan(1,cfg.N);
log.ftL = nan(1,cfg.N);
log.fnR = nan(1,cfg.N);
log.ftR = nan(1,cfg.N);
log.etaL = nan(1,cfg.N);
log.etaR = nan(1,cfg.N);
log.noFallMargin = nan(1,cfg.N);
log.tauL = nan(3,cfg.N);
log.tauR = nan(3,cfg.N);
log.rho = nan(4,cfg.N);
log.reward = nan(1,cfg.N);

for k = 1:cfg.N
    horizon = get_horizon_reference(ref, k, cfg);

    if k == 1
        previousEta = [0; 0];
    else
        previousEta = [log.etaL(k-1); log.etaR(k-1)];
        if any(isnan(previousEta))
            previousEta = [0; 0];
        end
    end

    [weights, rhoPrev] = update_mpc_weights( ...
        qL, qR, uLPrev, uRPrev, previousEta, ref, k, rhoPrev, cfg);
    log.rho(:,k) = weights.rho;

    costfun = @(z) mpc_cost(z, qL, qR, horizon, ...
        uLPrev, uRPrev, weights, cfg);
    nonlcon = @(z) mpc_constraints(z, qL, qR, cfg);

    [z, ~, exitflag] = fmincon(costfun, z0, [], [], [], [], ...
        bounds.lb, bounds.ub, nonlcon, cfg.optimizer);

    if exitflag <= 0
        warning('fmincon did not fully converge at step %d (exitflag %d).', ...
            k, exitflag);
    end

    [uL, uR, fL, fR] = unpack_mpc_step(z, cfg.H, 1);

    if cfg.useDisturbance
        uLReal = cfg.actGainL .* uL + disturbance.left(:,k);
        uRReal = cfg.actGainR .* uR + disturbance.right(:,k);
    else
        uLReal = uL;
        uRReal = uR;
    end

    qL = qL + cfg.dt*uLReal;
    qR = qR + cfg.dt*uRReal;

    pL = fk3R(qL, cfg.baseL, cfg.L1, cfg.L2, cfg.L3);
    pR = fk3R(qR, cfg.baseR, cfg.L1, cfg.L2, cfg.L3);

    JL = jacobian3R(qL, cfg.L1, cfg.L2, cfg.L3);
    JR = jacobian3R(qR, cfg.L1, cfg.L2, cfg.L3);
    tauGL = gravity3R(qL, cfg.L1, cfg.L2, cfg.L3, ...
        cfg.lc1, cfg.lc2, cfg.lc3, cfg.m1, cfg.m2, cfg.m3, cfg.g);
    tauGR = gravity3R(qR, cfg.L1, cfg.L2, cfg.L3, ...
        cfg.lc1, cfg.lc2, cfg.lc3, cfg.m1, cfg.m2, cfg.m3, cfg.g);

    fnL = fL(1); ftL = fL(2);
    fnR = fR(1); ftR = fR(2);
    etaL = abs(ftL)/(cfg.mu*max(fnL,1e-9));
    etaR = abs(ftR)/(cfg.mu*max(fnR,1e-9));
    noFallMargin = (ftL + ftR) - cfg.W;

    log.qL(:,k) = qL;
    log.qR(:,k) = qR;
    log.uL(:,k) = uL;
    log.uR(:,k) = uR;
    log.uL_real(:,k) = uLReal;
    log.uR_real(:,k) = uRReal;
    log.pL(:,k) = pL;
    log.pR(:,k) = pR;
    log.fnL(k) = fnL;
    log.ftL(k) = ftL;
    log.fnR(k) = fnR;
    log.ftR(k) = ftR;
    log.etaL(k) = etaL;
    log.etaR(k) = etaR;
    log.noFallMargin(k) = noFallMargin;
    log.tauL(:,k) = tauGL + JL.'*fL;
    log.tauR(:,k) = tauGR + JR.'*fR;

    eL = pL - ref.PL(:,k);
    eR = pR - ref.PR(:,k);
    ephiL = wrap_pi(sum(qL)-ref.phiL(k));
    ephiR = wrap_pi(sum(qR)-ref.phiR(k));

    reward = compute_reward(eL, eR, ephiL, ephiR, ...
        uL, uR, uLPrev, uRPrev, etaL, etaR, noFallMargin);
    log.reward(k) = reward;

    z0 = z;
    uLPrev = uL;
    uRPrev = uR;
end

result.log = log;
result.t = cfg.t;
result.P_L_ref = ref.PL;
result.P_R_ref = ref.PR;
result.x_obj = ref.xObj;
result.y_obj = ref.yObj;
result.rho_min = cfg.rhoMin;
result.rho_max = cfg.rhoMax;
result.useDDPGTuning = cfg.useAdaptiveTuning;  % legacy result field
result.useDisturbance = cfg.useDisturbance;
result.metrics = calculate_metrics(result, cfg.dt);

end
