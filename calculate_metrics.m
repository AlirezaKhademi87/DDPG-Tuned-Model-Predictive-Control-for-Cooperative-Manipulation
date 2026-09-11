function m = calculate_metrics(result, dt)
%CALCULATE_METRICS Performance quantities used in the paper comparison.

leftError = result.log.pL - result.P_L_ref;
rightError = result.log.pR - result.P_R_ref;
error2 = sum(leftError.^2,1) + sum(rightError.^2,1);
positionError = sqrt(error2);

m.tracking_RMSE = sqrt(mean(error2, 'omitnan'));
m.mean_position_error = mean(positionError, 'omitnan');
m.max_position_error = max(positionError, [], 'omitnan');

u2 = sum(result.log.uL.^2,1) + sum(result.log.uR.^2,1);
m.control_effort = sum(u2, 'omitnan') * dt;

uReal2 = sum(result.log.uL_real.^2,1) + sum(result.log.uR_real.^2,1);
m.actual_control_effort = sum(uReal2, 'omitnan') * dt;

tau2 = sum(result.log.tauL.^2,1) + sum(result.log.tauR.^2,1);
m.torque_effort = sum(tau2, 'omitnan') * dt;
m.max_eta = max([result.log.etaL(:); result.log.etaR(:)], [], 'omitnan');
m.mean_eta = mean([result.log.etaL(:); result.log.etaR(:)], 'omitnan');
m.mean_reward = mean(result.log.reward, 'omitnan');

m.slip_violation_area = sum( ...
    max(0,result.log.etaL-1).^2 + max(0,result.log.etaR-1).^2, ...
    'omitnan') * dt;
m.no_fall_violation_area = sum(max(0,-result.log.noFallMargin).^2, ...
    'omitnan') * dt;

duL = [diff(result.log.uL_real,1,2), zeros(3,1)];
duR = [diff(result.log.uR_real,1,2), zeros(3,1)];
m.actual_control_smoothness = sum( ...
    sum(duL.^2,1) + sum(duR.^2,1), 'omitnan');

% Readable aliases used by the plotting routines.
m.trackingRMSE = m.tracking_RMSE;
m.meanPositionError = m.mean_position_error;
m.maxPositionError = m.max_position_error;
m.controlEffort = m.control_effort;
end
