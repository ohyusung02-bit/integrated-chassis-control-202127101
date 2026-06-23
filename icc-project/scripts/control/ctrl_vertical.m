function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL [학생 작성] CDC (Continuous Damping Control) — per-wheel 감쇠 명령
%
%   Body-bounce / wheel-hop 모드 분리 및 ride comfort 개선을 위한 가변 감쇠.
%
%   Inputs:
%       suspState - struct, 각 wheel 의 sprung/unsprung velocity 등
%           .zs_dot(4)     - sprung mass velocity (위쪽 양수) [m/s]
%           .zu_dot(4)     - unsprung mass velocity [m/s]
%           .zs(4), .zu(4) - 변위 [m]
%       ctrlState - 내부 상태
%       CTRL      - .VER.cMin (≈ 500), .cMax (≈ 5000), .skyGain (≈ 2500)
%       dt        - sample time
%
%   Output:
%       dampingCmd - 4×1 damping coefficient [Ns/m]
%
%   요구사항:
%       1. Skyhook 기본:  c_i = skyGain · sign(zs_dot_i · (zs_dot_i - zu_dot_i))
%          (또는 force form: F = skyGain · zs_dot, F = c · (zs_dot - zu_dot))
%       2. cMin ≤ c ≤ cMax 제한
%       3. (옵션) Hybrid skyhook + groundhook
%       4. (옵션) body-bounce/wheel-hop 빈도 분리
%
%   힌트:
%       - Skyhook 의 핵심 원리: sprung mass 가 절대 좌표에서 정지하길 원함 → relative
%         damping 을 변조해 sprung velocity 를 줄임.
%       - 간단 force version: 항상 c = c_nom 으로 두고, (zs_dot · (zs_dot - zu_dot)) > 0
%         일 때만 c = cMax, 아니면 c = cMin (semi-active 의 on-off skyhook).

        %% 학생 구현: on-off skyhook + 연속형 보정 CDC

    % ---------- 0. 안전 처리 ----------
    if nargin < 4 || isempty(dt) || dt <= 0
        dt = 0.01;
    end

    if isempty(ctrlState)
        ctrlState = struct();
    end

    % ---------- 1. 기본 감쇠 파라미터 ----------
    cMin = 500;
    cMax = 5000;
    cNom = 1800;
    skyGain = 2500;

    if isfield(CTRL, 'VER')
        if isfield(CTRL.VER, 'cMin');    cMin = CTRL.VER.cMin;    end
        if isfield(CTRL.VER, 'cMax');    cMax = CTRL.VER.cMax;    end
        if isfield(CTRL.VER, 'skyGain'); skyGain = CTRL.VER.skyGain; end
    end

    % ---------- 2. suspension state 안전 처리 ----------
    zs_dot = zeros(4,1);
    zu_dot = zeros(4,1);

    if isfield(suspState, 'zs_dot')
        zs_dot = suspState.zs_dot(:);
    end
    if isfield(suspState, 'zu_dot')
        zu_dot = suspState.zu_dot(:);
    end

    if numel(zs_dot) ~= 4
        zs_dot = zeros(4,1);
    end
    if numel(zu_dot) ~= 4
        zu_dot = zeros(4,1);
    end

    relVel = zs_dot - zu_dot;

    % ---------- 3. skyhook 감쇠 계산 ----------
    dampingCmd = cNom * ones(4,1);

    for i = 1:4
        % semi-active skyhook 조건
        % zs_dot과 relVel 방향이 같으면 감쇠를 크게 줘서 차체 운동을 억제
        if zs_dot(i) * relVel(i) > 0
            cSky = skyGain * abs(zs_dot(i)) / (abs(relVel(i)) + 0.05);
            dampingCmd(i) = cNom + cSky;
        else
            dampingCmd(i) = cMin;
        end
    end

    % ---------- 4. 좌우/전후 급격한 차이 완화 ----------
    % 너무 큰 wheel별 차이는 tire load variation을 키울 수 있어서 평균과 섞음
    avgC = mean(dampingCmd);
    dampingCmd = 0.75 * dampingCmd + 0.25 * avgC;

    % ---------- 5. 최종 saturation ----------
    dampingCmd = min(max(dampingCmd, cMin), cMax);

end
