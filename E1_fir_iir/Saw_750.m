%% 生成 750 Hz 锯齿波并查看其频谱
clc; clear; close all;

%% 1. 参数设置
f0 = 750;            % 锯齿波基频 (Hz)
fs = 100e3;          % 采样率 (Hz)
T  = 0.02;           % 观察时长 (s)
N  = round(T*fs);    % 采样点数
t = (0:N-1)/fs;      % 时间轴

%% 2. 生成锯齿波
% 方法一：使用 sawtooth 函数（需 Signal Processing Toolbox）
% x = sawtooth(2*pi*f0*t);

% 方法二：手动生成，从 -1 线性上升到 1，然后跳变
x = 2*mod(f0*t, 1) - 1;

%% 3. 计算频谱 (FFT)
X = fft(x);
X = fftshift(X);                     % 零频移到中心
f = (-N/2:N/2-1) * (fs/N);           % 双边频率轴
mag = abs(X) / N;                    % 归一化幅度谱

% 只取正频率部分
idx = f >= 0;
f_pos   = f(idx);
mag_pos = mag(idx);

%% 4. 绘图
figure('Position',[100 100 1100 700],'Color','w');

% ---- 时域波形 ----
subplot(3,1,1);
plot(t*1e3, x, 'LineWidth', 1.2, 'Color', [0 0.45 0.74]);
xlim([0 4/f0*1e3]);                  % 显示 4 个周期
ylim([-1.3 1.3]);
xlabel('时间 (ms)');
ylabel('幅度');
title(sprintf('750 Hz 锯齿波时域波形 (fs = %.0f kHz)', fs/1e3));
grid on;

% ---- 幅度谱（线性，0~10 kHz）----
subplot(3,1,2);
stem(f_pos, mag_pos, 'filled', 'MarkerSize', 3, ...
     'Color', [0.85 0.33 0.10]);
xlim([0 10000]);
xlabel('频率 (Hz)');
ylabel('幅度');
title('幅度谱（线性）—— 所有整数次谐波，间隔 750 Hz');
grid on;

% ---- 幅度谱（dB，0~20 kHz）----
subplot(3,1,3);
mag_dB = 20*log10(mag_pos + eps);
plot(f_pos, mag_dB, 'LineWidth', 1.2, 'Color', [0.47 0.67 0.19]);
xlim([0 20000]);
ylim([-80 5]);
xlabel('频率 (Hz)');
ylabel('幅度 (dB)');
title('幅度谱（dB）—— 幅度按 1/n 衰减');
grid on;

%% 5. 打印前几个谐波频率
fprintf('基频 f0 = %.1f Hz\n', f0);
fprintf('理论谐波频率: ');
fprintf('%.0f  ', f0*(1:10));
fprintf('\n');