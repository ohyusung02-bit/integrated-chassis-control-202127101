function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL [학생 작성] 횡방향 통합 제어기 (AFS + ESC)
%
%   yaw rate 추종 (AFS) + slip angle 제한 (ESC) 통합 제어기를 설계하라.
%
%   Inputs:
%       yawRateRef - 목표 yaw rate [rad/s] (driver delta 로부터 bicycle model 로 계산됨)
%       yawRate    - 실제 yaw rate [rad/s]
%       slipAngle  - 차체 슬립 앵글 β [rad]
%       vx         - 종방향 속도 [m/s]
%       ctrlState  - 내부 상태 (.intError, .prevError, ... 자유롭게 확장 가능)
%       CTRL       - sim_params.m 에서 정의된 게인 (.LAT.Kp, .Ki, .Kd, .intMax)
%       LIM        - 한계값 (.MAX_STEER_ANGLE, .MAX_SLIP_ANGLE)
%       dt         - sample time [s]
%
%   Outputs:
%       deltaAdd.steerAngle - AFS 보조 조향각 [rad], 부호 driver delta 와 동일 방향
%       deltaAdd.yawMoment  - ESC 요청 yaw moment [Nm] (ctrl_coordinator 가 brake 차동으로 변환)
%       ctrlState           - 업데이트된 내부 상태
%
%   요구사항:
%       1. yaw rate 추종을 위한 보조 조향 (예: PID, LQR, pole placement, SMC 중 택일)
%       2. |slipAngle| > β_threshold 일 때 yaw moment 인가 (driver intent 와 반대 방향)
%       3. vx 적응 — 저속/고속 게인 differential (예: gain scheduling, LPV)
%       4. anti-windup, saturation 처리
%
%   금지:
%       - scenario id 분기 (예: 'A1 이면 X' 같은 hardcoding)
%       - LIM.MAX_STEER_ANGLE 위반
%       - global 변수 사용
%
%   힌트:
%       - PID 출발점은 sim_params.m 의 CTRL.LAT.Kp/Ki/Kd 값
%       - LQR 설계 시 Bicycle Model state-space (scripts/control/calc_bicycle_model.m 참조)
%       - β-limiter 는 다음 형태가 일반적:
%             if |β| > β_th
%                 M_z = -K_β · sign(β) · (|β| - β_th) · f(vx)
%       - speed scheduling: f(vx) = min(vx/v_ref, 2)

    %% 학생 구현: PID 기반 AFS + slip angle 기반 ESC

    % ---------- 0. 안전 처리 ----------
    if nargin < 8 || isempty(dt) || dt <= 0
        dt = 0.01;
    end

    if isempty(ctrlState)
        ctrlState = struct();
    end

    if ~isfield(ctrlState, 'intError')
        ctrlState.intError = 0;
    end
    if ~isfield(ctrlState, 'prevError')
        ctrlState.prevError = 0;
    end

    % ---------- 1. 게인 불러오기 ----------
    % sim_params.m에 값이 있으면 그 값을 쓰고, 없으면 기본값 사용
    Kp = 0.25;
    Ki = 0.02;
    Kd = 0.01;
    intMax = 0.5;

    if isfield(CTRL, 'LAT')
        if isfield(CTRL.LAT, 'Kp');     Kp = CTRL.LAT.Kp;       end
        if isfield(CTRL.LAT, 'Ki');     Ki = CTRL.LAT.Ki;       end
        if isfield(CTRL.LAT, 'Kd');     Kd = CTRL.LAT.Kd;       end
        if isfield(CTRL.LAT, 'intMax'); intMax = CTRL.LAT.intMax; end
    end

    % ---------- 2. 제한값 설정 ----------
    maxSteer = 0.045;        % AFS 보조 조향각 제한 [rad]
    betaMax  = deg2rad(5);  % slip angle 제한 기준 [rad]

    if isfield(LIM, 'MAX_STEER_ANGLE')
        maxSteer = min(maxSteer, 0.5 * LIM.MAX_STEER_ANGLE);
    end
    if isfield(LIM, 'MAX_SLIP_ANGLE')
        betaMax = LIM.MAX_SLIP_ANGLE;
    end

    betaTh = 0.60 * betaMax;     % slip angle 제어가 시작되는 문턱값

    % ---------- 3. 속도 스케줄링 ----------
    % 저속에서는 제어를 약하게, 고속에서는 적당히 강하게
    vxSafe = max(vx, 0.1);
    speedGain = min(max(vxSafe / 20, 0.4), 1.3);

    % ---------- 4. yaw rate 추종용 PID 제어 ----------
    error = yawRateRef - yawRate;

    ctrlState.intError = ctrlState.intError + error * dt;
    ctrlState.intError = min(max(ctrlState.intError, -intMax), intMax);

    dError = (error - ctrlState.prevError) / dt;
    ctrlState.prevError = error;

    steerCmd = 0.55 * speedGain * (Kp * error + Ki * ctrlState.intError + Kd * dError);

    % AFS 보조 조향각 saturation
    steerCmd = min(max(steerCmd, -maxSteer), maxSteer);

    % ---------- 5. slip angle 제한용 ESC yaw moment ----------
    yawMomentCmd = 0;

    if abs(slipAngle) > betaTh
        betaError = abs(slipAngle) - betaTh;

        % slip angle이 커질수록, 속도가 빠를수록 더 강한 yaw moment 인가
        Kbeta = 6000;
        yawMomentCmd = -Kbeta * sign(slipAngle) * betaError * speedGain;

        % yaw moment saturation
        maxYawMoment = 4000;
        yawMomentCmd = min(max(yawMomentCmd, -maxYawMoment), maxYawMoment);
    end

    % ---------- 6. 출력 ----------
    deltaAdd.steerAngle = steerCmd;
    deltaAdd.yawMoment  = yawMomentCmd;

end
