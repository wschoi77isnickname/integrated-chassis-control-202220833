function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL Longitudinal PI controller with simple ABS logic
%
% 설계 개요:
% 1) vxRef - vx 오차에 대한 PI 제어
% 2) 감속 중 과도한 제동을 막기 위한 brakeRatio 제한
% 3) jerk limit으로 Fx_total 변화율 제한
%
% 주의:
% 공개 runner 기준으로는 wheel slip ratio가 이 함수 입력으로 직접 들어오지 않으므로,
% 여기서는 ax 기반의 보수적 ABS-like modulation을 적용한다.

    if nargin < 7 || isempty(dt) || dt <= 0
        dt = 1e-3;
    end

    if isempty(ctrlState)
        ctrlState = struct();
    end
    if ~isfield(ctrlState, 'intError');  ctrlState.intError = 0; end
    if ~isfield(ctrlState, 'prevForce'); ctrlState.prevForce = 0; end

    Kp = getFieldSafe(CTRL.LON, 'Kp', 0.5);
    Ki = getFieldSafe(CTRL.LON, 'Ki', 0.05);
    intMax = getFieldSafe(CTRL.LON, 'intMax', 2000);

    maxAx = getFieldSafe(LIM, 'MAX_AX', 10.0);
    maxJerk = getFieldSafe(LIM, 'MAX_JERK', 50.0);

    m = 1500;   % default mass if VEH is not passed to this function

    err = vxRef - vx;

    ctrlState.intError = ctrlState.intError + err * dt;
    ctrlState.intError = sat(ctrlState.intError, -intMax, intMax);

    FxRaw = m * (Kp * err + Ki * ctrlState.intError);

    % acceleration/deceleration force limit
    FxRaw = sat(FxRaw, -m * maxAx, m * maxAx);

    % ABS-like conservative modulation:
    % 감속도가 너무 크면 제동력을 줄여 slip divergence를 방지
    if ax < -0.85 * maxAx && FxRaw < 0
        FxRaw = 0.65 * FxRaw;
    elseif ax < -0.65 * maxAx && FxRaw < 0
        FxRaw = 0.80 * FxRaw;
    end

    % jerk limit: dF/dt <= m * maxJerk
    dFmax = m * maxJerk * dt;
    FxCmd = sat(FxRaw, ctrlState.prevForce - dFmax, ctrlState.prevForce + dFmax);

    ctrlState.prevForce = FxCmd;

    forceCmd.Fx_total = FxCmd;

    if FxCmd < 0
        forceCmd.brakeRatio = sat(abs(FxCmd) / (m * maxAx), 0, 1);
    else
        forceCmd.brakeRatio = 0;
    end
end

function y = sat(x, lo, hi)
    y = min(max(x, lo), hi);
end

function v = getFieldSafe(S, name, defaultVal)
    if isstruct(S) && isfield(S, name)
        v = S.(name);
    else
        v = defaultVal;
    end
end