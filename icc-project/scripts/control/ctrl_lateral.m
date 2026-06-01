function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL Integrated lateral controller: AFS + ESC
%
% 설계 개요:
% 1) AFS: yaw-rate tracking PID
% 2) ESC: slip angle beta limiter
% 3) Gain scheduling: high speed에서 AFS 조향 개입은 줄이고 ESC yaw moment 개입은 증가
% 4) Anti-windup + saturation 적용
%
% Inputs:
%   yawRateRef : reference yaw rate [rad/s]
%   yawRate    : measured yaw rate [rad/s]
%   slipAngle  : body slip angle beta [rad]
%   vx         : longitudinal speed [m/s]
%   ctrlState  : controller internal state
%   CTRL, LIM  : parameter structs
%   dt         : sample time [s]
%
% Outputs:
%   deltaAdd.steerAngle : additional front steering [rad]
%   deltaAdd.yawMoment  : requested yaw moment [Nm]

    if nargin < 8 || isempty(dt) || dt <= 0
        dt = 1e-3;
    end

    if isempty(ctrlState)
        ctrlState = struct();
    end
    if ~isfield(ctrlState, 'intError');   ctrlState.intError = 0; end
    if ~isfield(ctrlState, 'prevError');  ctrlState.prevError = 0; end
    if ~isfield(ctrlState, 'prevSteer');  ctrlState.prevSteer = 0; end
    if ~isfield(ctrlState, 'prevMz');     ctrlState.prevMz = 0; end

    % ---------- 1. 기본 파라미터 ----------
    Kp = getFieldSafe(CTRL.LAT, 'Kp', 1.0);
    Ki = getFieldSafe(CTRL.LAT, 'Ki', 0.1);
    Kd = getFieldSafe(CTRL.LAT, 'Kd', 0.05);

    intMax = getFieldSafe(CTRL.LAT, 'intMax', 5.0);

    maxSteer = getFieldSafe(LIM, 'MAX_STEER_ANGLE', deg2rad(36));
    maxSteerRate = getFieldSafe(LIM, 'MAX_STEER_RATE', deg2rad(33));

    betaTh = deg2rad(3.0);
    betaHard = deg2rad(7.0);

    maxMz = 800;          % [Nm] ESC yaw moment limit
    maxMzRate = 60000;     % [Nm/s] yaw moment rate limit

    vxEff = max(abs(vx), 0.5);

    % 고속일수록 AFS 직접 조향은 줄이고 ESC 안정화는 키움
    steerSchedule = 1 / max(1.0, vxEff / 18.0);
    escSchedule   = min(max(vxEff / 15.0, 0.5), 2.2);

    % ---------- 2. AFS: yaw-rate PID ----------
    err = yawRateRef - yawRate;

    ctrlState.intError = ctrlState.intError + err * dt;
    ctrlState.intError = sat(ctrlState.intError, -intMax, intMax);

    derr = (err - ctrlState.prevError) / dt;

    steerRaw = steerSchedule * (Kp * err + Ki * ctrlState.intError + Kd * derr);

    % slip angle이 이미 큰 경우, AFS가 더 미끄러지게 만들지 않도록 감쇠
    betaAbs = abs(slipAngle);
    if betaAbs > betaTh
        steerRaw = 0.55 * steerRaw;
    end
    if betaAbs > betaHard
        steerRaw = 0.25 * steerRaw;
    end

    % 조향각 제한
    steerCmd = sat(steerRaw, -0.105 * maxSteer, 0.105 * maxSteer);

    % 조향 rate 제한
    dSteerMax = maxSteerRate * dt;
    steerCmd = sat(steerCmd, ctrlState.prevSteer - dSteerMax, ctrlState.prevSteer + dSteerMax);

    % ---------- 3. ESC: slip angle limiter + yaw-rate damping ----------
    Kbeta = 16000;   % [Nm/rad]
    Kr    = 200;    % [Nm/(rad/s)]

    if betaAbs > betaTh
        betaExcess = betaAbs - betaTh;

        % beta가 양수면 반대 방향 yaw moment를 줘서 옆미끄러짐 감소
        Mz_beta = -Kbeta * sign(slipAngle) * betaExcess * escSchedule;
    else
        Mz_beta = 0;
    end

    % yaw-rate tracking 보조 yaw moment
    Mz_yaw = Kr * err * escSchedule;

    % slip angle 안정성을 더 우선
    if betaAbs > betaTh
        MzRaw = Mz_beta + 0.35 * Mz_yaw;
    else
        MzRaw = 0.55 * Mz_yaw;
    end

    MzCmd = sat(MzRaw, -maxMz, maxMz);

    % yaw moment rate 제한
    dMzMax = maxMzRate * dt;
    MzCmd = sat(MzCmd, ctrlState.prevMz - dMzMax, ctrlState.prevMz + dMzMax);

    % ---------- 4. 상태 저장 ----------
    ctrlState.prevError = err;
    ctrlState.prevSteer = steerCmd;
    ctrlState.prevMz = MzCmd;

    deltaAdd.steerAngle = steerCmd;
    deltaAdd.yawMoment = MzCmd;
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