function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL Semi-active CDC using on-off skyhook logic
%
% 설계 개요:
% 1) Skyhook: sprung mass velocity를 줄이는 방향이면 cMax
% 2) 반대 방향이면 cMin
% 3) 급격한 damping 변화 방지를 위해 rate smoothing 적용

    if nargin < 4 || isempty(dt) || dt <= 0
        dt = 1e-3;
    end

    if isempty(ctrlState)
        ctrlState = struct();
    end
    if ~isfield(ctrlState, 'prevC')
        ctrlState.prevC = [];
    end

    cMin = getFieldSafe(CTRL.VER, 'cMin', 500);
    cMax = getFieldSafe(CTRL.VER, 'cMax', 5000);
    cNom = 0.5 * (cMin + cMax);

    zs_dot = getFieldSafe(suspState, 'zs_dot', zeros(4, 1));
    zu_dot = getFieldSafe(suspState, 'zu_dot', zeros(4, 1));

    zs_dot = zs_dot(:);
    zu_dot = zu_dot(:);

    if numel(zs_dot) ~= 4
        zs_dot = zeros(4, 1);
    end
    if numel(zu_dot) ~= 4
        zu_dot = zeros(4, 1);
    end

    relVel = zs_dot - zu_dot;

    dampingRaw = cNom * ones(4, 1);

    for i = 1:4
        % Skyhook condition:
        % sprung velocity와 relative velocity가 같은 방향이면 큰 감쇠
        if zs_dot(i) * relVel(i) > 0
            dampingRaw(i) = cMax;
        else
            dampingRaw(i) = cMin;
        end
    end

    % damping 변화 rate smoothing
    if isempty(ctrlState.prevC) || numel(ctrlState.prevC) ~= 4
        ctrlState.prevC = cNom * ones(4, 1);
    end

    maxRate = 80000;  % [Ns/m/s]
    dcMax = maxRate * dt;

    dampingCmd = zeros(4, 1);
    for i = 1:4
        dampingCmd(i) = sat(dampingRaw(i), ctrlState.prevC(i) - dcMax, ctrlState.prevC(i) + dcMax);
    end

    dampingCmd = min(max(dampingCmd, cMin), cMax);
    ctrlState.prevC = dampingCmd;
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