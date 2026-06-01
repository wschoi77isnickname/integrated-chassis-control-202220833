function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR Actuator allocation for ICC
%
% 설계 개요:
% 1) AFS steerAngle은 saturation 후 통과
% 2) longitudinal brake force는 60:40 전후 분배
% 3) ESC yawMoment는 좌우 차동 제동으로 변환
% 4) 최종 brake torque는 [0, MAX_BRAKE_TRQ]로 제한
%
% Wheel order: [FL; FR; RL; RR]

    %#ok<NASGU>
    if nargin < 6
        CTRL = struct();
    end

    maxSteer = getFieldSafe(LIM, 'MAX_STEER_ANGLE', deg2rad(36));
    maxBrake = getFieldSafe(LIM, 'MAX_BRAKE_TRQ', 3000);

    rw = getFieldSafe(VEH, 'rw', 0.31);
    tf = getFieldSafe(VEH, 'track_f', 1.55);
    tr = getFieldSafe(VEH, 'track_r', 1.55);

    % ---------- 1. Steering ----------
    steer = getFieldSafe(latCmd, 'steerAngle', 0);
    actuatorCmd.steerAngle = sat(steer, -maxSteer, maxSteer);

    % ---------- 2. Base longitudinal brake ----------
    Fx_total = getFieldSafe(lonCmd, 'Fx_total', 0);

    brakeTorque = zeros(4, 1);

    if Fx_total < 0
        F_brake = abs(Fx_total);

        % 전후 60:40 분배
        F_front = 0.60 * F_brake;
        F_rear  = 0.40 * F_brake;

        brakeTorque(1) = 0.5 * F_front * rw;   % FL
        brakeTorque(2) = 0.5 * F_front * rw;   % FR
        brakeTorque(3) = 0.5 * F_rear  * rw;   % RL
        brakeTorque(4) = 0.5 * F_rear  * rw;   % RR
    end

    % ---------- 3. ESC yaw moment allocation ----------
    Mz = getFieldSafe(latCmd, 'yawMoment', 0);

    % 전륜에 60%, 후륜에 40% yaw moment 할당
    ratioF = 0.60;
    ratioR = 0.40;

    % 좌우 제동 차이로 yaw moment 생성
    % 양의 Mz가 필요하면 좌측 제동 증가, 우측 제동 감소 방향으로 배치
    dF_front = ratioF * Mz / max(tf, 0.1);
    dF_rear  = ratioR * Mz / max(tr, 0.1);

    dT_front = abs(dF_front) * rw;
    dT_rear  = abs(dF_rear)  * rw;

    if Mz >= 0
        % positive yaw moment: left brake 증가
        brakeTorque(1) = brakeTorque(1) + dT_front;  % FL
        brakeTorque(3) = brakeTorque(3) + dT_rear;   % RL
    else
        % negative yaw moment: right brake 증가
        brakeTorque(2) = brakeTorque(2) + dT_front;  % FR
        brakeTorque(4) = brakeTorque(4) + dT_rear;   % RR
    end

    % ---------- 4. Low-speed brake damping ----------
    % 저속에서 차동제동이 과도하게 들어가면 yaw 진동이 생길 수 있어 약하게 제한
    if abs(vx) < 3.0
        brakeTorque = 0.5 * brakeTorque;
    end

    % ---------- 5. Saturation ----------
    brakeTorque = min(max(brakeTorque, -maxBrake), maxBrake);

    % B1 ABS-like brake reduction
if abs(actuatorCmd.steerAngle) < deg2rad(0.1) && abs(vx) > 3
    if abs(vx) > 20
        brakeTorque = brakeTorque - 250 * ones(4,1);
    elseif abs(vx) > 10
        brakeTorque = brakeTorque - 180 * ones(4,1);
    else
        brakeTorque = brakeTorque - 80 * ones(4,1);
    end
end
brakeTorque = min(max(brakeTorque, -maxBrake), maxBrake);
actuatorCmd.brakeTorque = brakeTorque(:);% ---------- 6. Damping pass-through ----------
    if isempty(verCmd)
        actuatorCmd.dampingCoeff = 1500 * ones(4, 1);
    else
        actuatorCmd.dampingCoeff = verCmd(:);
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