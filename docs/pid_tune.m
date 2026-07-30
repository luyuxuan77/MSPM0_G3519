%% =========================================================================
%  pid_tune.m  —  从串口采集的数据自动整定 ANGLE 航向环 PID
%  用法:
%    1. 固件里 ANGLE_SYSID=1, 烧录。进 HOLD ANGLE 页按 K1 START,
%       车会每3秒左右转40度来回摆(方波激励)。让它摆 20~30 秒。
%    2. 用串口助手把打印的 CSV 行存成文本文件, 例如 data.txt。
%    3. 把下面 DATA_FILE 改成你的文件路径, 在 MATLAB 里运行本脚本。
%    4. 命令行窗口会打印要抄回 main.c 的 3 个增益。
%  依赖: System Identification Toolbox + Control System Toolbox (你都装了)
%% =========================================================================
clear; clc; close all;

DATA_FILE = 'E:\Cuey\Keil\M0g3519\Car_sum\pro_bigger_car\docs\data.txt';     % ← 改成你的数据文件路径

%% ---- 1. 读取并清洗数据 ----
% 文件里可能混有 "=== SYSTEM RESET ===" 等非数据行, 只保留5列纯数字的行。
lines = readlines(DATA_FILE);           % 逐行读入 (含杂行)
M = [];
for i = 1:numel(lines)
    s = strtrim(lines(i));
    % 只保留形如 "数,数,数,数,数" 的行 (允许负号/小数)
    if ~isempty(regexp(s, '^-?\d[\d\.\-,eE]*$', 'once'))
        v = sscanf(s, '%f,%f,%f,%f,%f')';
        if numel(v) == 5, M = [M; v]; end %#ok<AGROW>
    end
end
if size(M,1) < 50, error('有效数据太少(%d行), 检查文件格式/多采点数据', size(M,1)); end

t_ms = M(:,1);  setp = M(:,2);  yaw = M(:,3);  err = M(:,4);  steer = M(:,5);

t = (t_ms - t_ms(1)) / 1000;            % 秒
Ts = median(diff(t));                    % 采样周期(应≈0.01)
fprintf('读入 %d 个样本, Ts=%.4fs (%.0fHz)\n', numel(t), Ts, 1/Ts);

% 数据质量检查: steer 饱和过多则辨识不可信
STEER_LIM = 50;
sat = sum(abs(steer) >= STEER_LIM*0.99) / numel(steer) * 100;
fprintf('steer 饱和占比 = %.1f%%\n', sat);
if sat > 30
    warning(['steer 饱和 %.0f%% 过高! 辨识会不准。\n' ...
             '请减小激励幅度或采集增益, 让 steer 待在 ±%d 以内再重采。'], sat, STEER_LIM);
end

%% ---- 2. 系统辨识: steer→yaw_rate(一阶,稳健), 再加积分器成 steer→yaw ----
% 直接辨识 steer→yaw 会把慢漂移拟合成假极点(不可靠)。
% 物理上 steer→yaw_rate 是一阶, yaw 是其积分。所以先辨识速率, 再手工加积分器。
yr = movmean([0; diff(yaw)]/Ts, 3);      % yaw_rate(deg/s), 3点平滑去噪
u = steer - mean(steer);
y = yr - mean(yr);
dat = iddata(y, u, Ts);

best = -inf; Grate = [];
for np = 1:2
    try
        m = tfest(dat, np, 0);
        f = m.Report.Fit.FitPercent;
        fprintf('  steer->yaw_rate [%dp] 拟合度 %.1f%%\n', np, f);
        if f > best, best = f; Grate = tf(m); end
    catch
    end
end
if isempty(Grate), error('辨识失败, 激励不够, 让车多蛇形几个来回再采'); end
fprintf('速率模型拟合度 %.1f%%, 稳态增益 K=%.3f deg/s/单位\n', best, dcgain(Grate));
if best < 55
    warning('拟合度<55%%, 结果只作起点参考, 建议实车再微调');
end
Gc = Grate * tf(1,[1 0]);                % steer→yaw = 速率模型 × 积分器

%% ---- 3. 自动整定 PIDF (带微分滤波; 纯PI对积分型对象会剧烈振荡) ----
WC = 3;                                   % 目标带宽(rad/s): 越大越快越猛, 2~5之间调
opt = pidtuneOptions('PhaseMargin', 60);
C = pidtune(Gc, 'PIDF', WC, opt);         % PIDF = P+I+带滤波的D
Kp = C.Kp;  Ki = C.Ki;  Kd = C.Kd;  Tf = C.Tf;
fprintf('\n连续PIDF(wc=%d): Kp=%.3f Ki=%.3f Kd=%.3f Tf=%.4f\n', WC, Kp, Ki, Kd, Tf);

%% ---- 4. 换算成固件里的宏 ----
%  固件(10ms控制周期): isum+=err*KI_fw(≈Ki∫e); D经一阶低通 alpha=ANGLE_D_LPF
%    => KP_fw=Kp; KI_fw=Ki*Ts; KD_fw=Kd; ANGLE_D_LPF≈Ts/(Tf+Ts)
Ts_ctrl = 0.01;                           % 固件控制周期(10ms), 与采样Ts可能不同
KP_fw = Kp;
KI_fw = Ki * Ts_ctrl;
KD_fw = Kd;
D_LPF = Ts_ctrl / (Tf + Ts_ctrl);
if D_LPF > 1, D_LPF = 1; end

fprintf('\n======== 抄回 main.c 的 ANGLE 增益 ========\n');
fprintf('#define ANGLE_YAW_KP    %10.4ff\n', KP_fw);
fprintf('#define ANGLE_YAW_KI    %10.4ff\n', KI_fw);
fprintf('#define ANGLE_YAW_KD    %10.4ff\n', KD_fw);
fprintf('#define ANGLE_D_LPF     %10.4ff\n', D_LPF);
fprintf('==========================================\n');
fprintf('(GO 100cm 的 GO100_YAW_* 同理, 想用一样的可一起抄)\n');
fprintf('(想更快改WC=4/5重跑; 更稳改WC=2)\n');

%% ---- 5. 仿真闭环阶跃, 预测整定效果 ----
sys_cl = feedback(C*Gc, 1);
figure('Name','整定后闭环阶跃响应');
step(sys_cl, 5); grid on;
title(sprintf('闭环阶跃 (Kp=%.2f Ki=%.2f Kd=%.2f)', Kp, Ki, Kd));
info = stepinfo(sys_cl);
fprintf('\n预测性能: 超调 %.1f%%, 调节时间 %.2fs, 上升时间 %.2fs\n', ...
        info.Overshoot, info.SettlingTime, info.RiseTime);

%% ---- 6. 实测原始响应: steer输入 vs yaw输出 ----
figure('Name','实测激励与响应');
subplot(2,1,1);
plot(t, steer, 'r', 'LineWidth', 1); grid on;
ylabel('steer (输入)'); title('开环方波转向 → yaw 响应');
subplot(2,1,2);
plot(t, yaw, 'b', 'LineWidth', 1); grid on;
xlabel('时间 (s)'); ylabel('yaw (deg, 输出)');

