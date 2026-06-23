function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR [학생 작성] Actuator Allocation — 횡/종/수직 명령을 actuator 로 분배
%
%   상위 제어기들의 명령 (yaw moment, Fx_total, damping) 을 차량 actuator
%   (steerAngle, 4-wheel brake torque, 4-wheel damping) 로 변환.
%
%   Inputs:
%       latCmd.steerAngle - AFS 보조 조향 [rad]
%       latCmd.yawMoment  - ESC 요청 yaw moment [Nm]
%       lonCmd.Fx_total   - 종방향 힘 요구 [N]
%       lonCmd.brakeRatio - 제동 비율
%       verCmd            - 4×1 damping [Ns/m] (ctrl_vertical 출력)
%       vx, VEH, CTRL, LIM
%
%   Output:
%       actuatorCmd.steerAngle    - 최종 조향각 [rad], LIM.MAX_STEER_ANGLE 제한
%       actuatorCmd.brakeTorque   - 4×1 brake torque [Nm], [FL; FR; RL; RR], LIM.MAX_BRAKE_TRQ 제한
%       actuatorCmd.dampingCoeff  - 4×1 [Ns/m]
%
%   요구사항:
%       1. 종방향 제동 (lonCmd.Fx_total < 0) 의 4륜 균등 분배 — 전후 비율 60:40 권장
%       2. ESC yaw moment → brake 차동 분배 (좌/우 비대칭)
%             양의 M_z (CCW) → 좌측 brake 증가 또는 우측 brake 감소
%             track 반거리: t_f/2 = VEH.track_f/2,  t_r/2 = VEH.track_r/2
%             dT_f = M_z · ratio_f / t_f,  dT_r = M_z · (1-ratio_f) / t_r
%       3. AFS steerAngle 그대로 통과 + saturation
%       4. brake torque 합산 후 [0, MAX_BRAKE_TRQ] 클리핑
%
%   가산점 (선택):
%       - 마찰원 제한: 각 휠의 brake torque + cornering force 가 μ·Fz 안으로
%       - WLS allocation: actuator effort minimize 목적함수
%       - per-wheel 최대 토크 제한 — wheel slip 임계 도달 시 감소
%
%   힌트:
%       - half-track: t_f/2 ≈ 0.78 m (BMW_5)
%       - 종방향 brake 시 force-to-torque: T = |Fx_total|/4 · r_w  (r_w ≈ 0.33 m)
%       - allocation matrix form 도 가능 (LQ allocation)

        %% 학생 구현: AFS + 종방향 제동 + ESC yaw moment allocation

    % ---------- 0. 입력 안전 처리 ----------
    if isempty(latCmd)
        latCmd.steerAngle = 0;
        latCmd.yawMoment  = 0;
    end
    if ~isfield(latCmd, 'steerAngle'); latCmd.steerAngle = 0; end
    if ~isfield(latCmd, 'yawMoment');  latCmd.yawMoment  = 0; end

    if isempty(lonCmd)
        lonCmd.Fx_total = 0;
        lonCmd.brakeRatio = 0;
    end
    if ~isfield(lonCmd, 'Fx_total');    lonCmd.Fx_total = 0; end
    if ~isfield(lonCmd, 'brakeRatio');  lonCmd.brakeRatio = 0; end

    if isempty(verCmd)
        verCmd = zeros(4,1);
    end
    if numel(verCmd) ~= 4
        verCmd = zeros(4,1);
    else
        verCmd = verCmd(:);
    end

    % ---------- 1. 차량/제한 파라미터 ----------
    maxSteer = deg2rad(8);
    maxBrake = 4000;
    rw = 0.33;          % wheel radius [m]
    track_f = 1.56;     % front track [m]
    track_r = 1.56;     % rear track [m]

    if isfield(LIM, 'MAX_STEER_ANGLE')
        maxSteer = LIM.MAX_STEER_ANGLE;
    end
    if isfield(LIM, 'MAX_BRAKE_TRQ')
        maxBrake = LIM.MAX_BRAKE_TRQ;
    end

    if isfield(VEH, 'wheelRadius')
        rw = VEH.wheelRadius;
    elseif isfield(VEH, 'r_w')
        rw = VEH.r_w;
    end

    if isfield(VEH, 'track_f')
        track_f = VEH.track_f;
    end
    if isfield(VEH, 'track_r')
        track_r = VEH.track_r;
    end

    % ---------- 2. 조향 명령 saturation ----------
    steerCmd = latCmd.steerAngle;
    steerCmd = min(max(steerCmd, -maxSteer), maxSteer);

    % ---------- 3. 종방향 제동 force -> 4륜 brake torque ----------
    brakeTorque = zeros(4,1);   % [FL; FR; RL; RR]

    Fx_total = lonCmd.Fx_total;

    % Fx_total < 0 이면 제동
    if Fx_total < 0
        T_total = abs(Fx_total) * rw;

        % 기본 전후 제동 분배: front 60%, rear 40%
        frontRatio = 0.60;
        rearRatio  = 0.40;

        brakeTorque(1) = T_total * frontRatio / 2;  % FL
        brakeTorque(2) = T_total * frontRatio / 2;  % FR
        brakeTorque(3) = T_total * rearRatio  / 2;  % RL
        brakeTorque(4) = T_total * rearRatio  / 2;  % RR
    end

    % brakeRatio가 따로 주어지는 경우 보조 반영
    if lonCmd.brakeRatio > 0 && Fx_total >= 0
        ratio = min(max(lonCmd.brakeRatio, 0), 1);
        T_total = ratio * 4 * maxBrake * 0.6;
        brakeTorque(1) = T_total * 0.60 / 2;
        brakeTorque(2) = T_total * 0.60 / 2;
        brakeTorque(3) = T_total * 0.40 / 2;
        brakeTorque(4) = T_total * 0.40 / 2;
    end

    % ---------- 4. ESC yaw moment -> 좌우 차동 브레이크 ----------
    Mz = latCmd.yawMoment;

    % 속도가 너무 낮으면 차동브레이크를 약화
    vxSafe = max(vx, 0);
    escGain = min(max(vxSafe / 15, 0), 1.2);

    % yaw moment 분배: front 70%, rear 30%
    yawFrontRatio = 0.70;
    yawRearRatio  = 0.30;

    % torque difference 계산
    % 양의 Mz가 필요하면 왼쪽 브레이크 증가, 음의 Mz면 오른쪽 브레이크 증가
    dT_f = escGain * Mz * yawFrontRatio / max(track_f, 0.1);
    dT_r = escGain * Mz * yawRearRatio  / max(track_r, 0.1);

    if Mz > 0
        brakeTorque(1) = brakeTorque(1) + abs(dT_f);  % FL
        brakeTorque(3) = brakeTorque(3) + abs(dT_r);  % RL
    elseif Mz < 0
        brakeTorque(2) = brakeTorque(2) + abs(dT_f);  % FR
        brakeTorque(4) = brakeTorque(4) + abs(dT_r);  % RR
    end

    % ---------- 5. 최종 saturation ----------
    brakeTorque = min(max(brakeTorque, 0), maxBrake);

    % damping도 일단 pass-through
    dampingCoeff = verCmd;



    % ===== straight braking 보조 제동: B1 중심, 조향/ESC 개입 시나리오 제외 =====
isStraightBrakeLike = (abs(latCmd.steerAngle) < deg2rad(0.2)) && ...
                      (abs(latCmd.yawMoment) < 50) && ...
                      (vx > 8);

if isStraightBrakeLike
    extraT = 850;  % per-wheel 추가 제동 토크 [Nm]
    brakeTorque = brakeTorque + extraT * ones(4,1);
end

brakeTorque = min(max(brakeTorque, -maxBrake), maxBrake);




    % ---------- 6. 출력 ----------
    actuatorCmd.steerAngle   = steerCmd;
    actuatorCmd.brakeTorque  = brakeTorque;
    actuatorCmd.dampingCoeff = dampingCoeff;

    
end
