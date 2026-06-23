function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL [학생 작성] 종방향 제어기 (속도 추종 + ABS)
%
%   속도 추종 (cruise/decel) 과 anti-lock braking (slip ratio limiting) 을 통합.
%
%   Inputs:
%       vxRef     - 목표 종방향 속도 [m/s]
%       vx        - 실제 종방향 속도 [m/s]
%       ax        - 종가속도 [m/s²]
%       ctrlState - 내부 상태 (.intError, .prevForce, .wheelSlip(4) 추가 가능)
%       CTRL      - .LON.Kp, .Ki, .intMax
%       LIM       - .MAX_AX, .MAX_JERK, .MAX_BRAKE_TRQ
%       dt        - sample time
%
%   Outputs:
%       forceCmd.Fx_total   - 총 종방향 힘 요구 [N], 양수 가속 / 음수 제동
%       forceCmd.brakeRatio - 제동 비율 (0: 가속, 1: 전제동) — 차후 coordinator 가 brake 토크로 변환
%       ctrlState           - 업데이트
%
%   요구사항:
%       1. 속도 추종 PI 제어
%       2. ABS — wheel slip ratio |κ| > 0.12 일 때 brake force 감소 (slip-limit 또는 bang-bang)
%       3. 저크 제한 (LIM.MAX_JERK · m 으로 force 미분 cap)
%       4. anti-windup
%
%   주의:
%       - 본 함수는 wheel slip 정보가 직접 입력으로 들어오지 않음. 학생은 runner 가 매 step
%         result.tire.{FL,FR,RL,RR}.slipRatio 에 기록하는 값을 ctrlState 에 캐시하는 식으로
%         설계할 수 있음. 또는 ctrl_coordinator 에서 ABS 모듈레이션 (다른 설계 선택).
%       - 본 과제 시나리오 (B1) 는 vxRef 일정 — PID 속도 추종보다 ABS 가 핵심.
%
%   힌트:
%       - slip ratio κ = (ω·r_w - vx) / max(vx, 0.1)
%       - ABS 작동 조건: vehicle 감속 중 (ax < 0) AND |κ| > κ_target (≈0.12)
%       - Bang-bang ABS: brake_cmd = brake_cmd · 0.5 일 때 |κ| > κ_target

     %% 학생 구현: PI 속도 추종 + 제동 force 제한 + jerk 제한

    % ---------- 0. 안전 처리 ----------
    if nargin < 7 || isempty(dt) || dt <= 0
        dt = 0.01;
    end

    if isempty(ctrlState)
        ctrlState = struct();
    end

    if ~isfield(ctrlState, 'intError')
        ctrlState.intError = 0;
    end
    if ~isfield(ctrlState, 'prevForce')
        ctrlState.prevForce = 0;
    end

    % ---------- 1. 기본 파라미터 ----------
    m = 1800;           % BMW 5급 차량 질량 근사 [kg]
    Kp = 900;           % 속도 오차 비례 게인
    Ki = 80;            % 속도 오차 적분 게인
    intMax = 20;        % 적분항 제한

    maxAx = 7.0;        % 최대 감속 한계 [m/s^2]
    maxJerk = 200;       % 저크 제한 [m/s^3]

    if isfield(CTRL, 'LON')
        if isfield(CTRL.LON, 'Kp');     Kp = CTRL.LON.Kp;       end
        if isfield(CTRL.LON, 'Ki');     Ki = CTRL.LON.Ki;       end
        if isfield(CTRL.LON, 'intMax'); intMax = CTRL.LON.intMax; end
    end

    if isfield(LIM, 'MAX_AX')
        maxAx = abs(LIM.MAX_AX);
    end
    if isfield(LIM, 'MAX_JERK')
        maxJerk = abs(LIM.MAX_JERK);
    end

    % ---------- 2. 속도 오차 기반 PI 제어 ----------
    % vxRef < vx 이면 감속/제동 명령이 나와야 하므로 error는 음수
    error = vxRef - vx;

    ctrlState.intError = ctrlState.intError + error * dt;
    ctrlState.intError = min(max(ctrlState.intError, -intMax), intMax);

    Fx_raw = Kp * error + Ki * ctrlState.intError;

    % ---------- 3. 제동 상황 보강 ----------
    % 목표 속도가 현재보다 충분히 낮으면 확실히 제동
    if error < -0.2
        % 기본적으로 요구 감속을 속도 오차에 비례시킴
        desiredDecel = min(max(abs(error) * 1.5, 4.0), maxAx);

        % 너무 저속에서는 제동을 줄여서 정지 직전 불안정 완화
        lowSpeedScale = min(max(vx / 5.0, 0.25), 1.0);

        Fx_brake = -m * desiredDecel * lowSpeedScale;

        % PI 결과와 목표 감속 기반 결과 중 더 강한 제동 사용
        Fx_raw = min(Fx_raw, Fx_brake);
    end

    % ---------- 4. 간단한 ABS 유사 감속 제한 ----------
    % wheel slip 입력이 없으므로 ax가 과도하게 음수이면 제동 force를 완화
    % ax < -8 m/s^2 수준이면 타이어가 과하게 물린 것으로 보고 제동 완화
    if ax < -8.0
        Fx_raw = 0.65 * Fx_raw;
    elseif ax < -7.0
        Fx_raw = 0.80 * Fx_raw;
    end

    % ---------- 5. 전체 force saturation ----------
    Fx_min = -m * maxAx;       % 최대 제동 force
    Fx_max =  0.15 * m * maxAx; % 가속 명령은 약하게 제한

    Fx_sat = min(max(Fx_raw, Fx_min), Fx_max);

    % ---------- 6. jerk 제한 ----------
    dFmax = m * maxJerk * dt;
    Fx_cmd = min(max(Fx_sat, ctrlState.prevForce - dFmax), ctrlState.prevForce + dFmax);

    ctrlState.prevForce = Fx_cmd;

    % ---------- 7. brakeRatio 계산 ----------
    if Fx_cmd < 0
        brakeRatio = min(max(abs(Fx_cmd) / abs(Fx_min), 0), 1);
    else
        brakeRatio = 0;
    end

    % ---------- 8. 출력 ----------
    forceCmd.Fx_total   = Fx_cmd;
    forceCmd.brakeRatio = brakeRatio;

end
