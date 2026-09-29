%% 生成 750 Hz 方波并查看其频谱
clc; clear; close all;

%% 1. 参数设置
f0 = 750;            % 方波基频 (Hz)
fs = 100e3;          % 采样率 (Hz)，远大于 2*f0，保证谐波可见
T  = 0.02;           % 观察时长 (s)
N  = round(T*fs);    % 采样点数

t = (0:N-1)/fs;      % 时间轴

%% 2. 生成方波
% 方法一：使用内置 square 函数（需 Signal Processing Toolbox）
x = square(2*pi*f0*t);

% 方法二：若无工具箱，可用下面这行代替上面一行
% x = sign(sin(2*pi*f0*t));

%% 3. 计算频谱 (FFT)
X = fft(x);
X = fftshift(X);                     % 将零频移到中心
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
title(sprintf('750 Hz 方波时域波形 (fs = %.0f kHz)', fs/1e3));
grid on;

% ---- 幅度谱（线性，0~10 kHz）----
subplot(3,1,2);
stem(f_pos, mag_pos, 'filled', 'MarkerSize', 3, ...
     'Color', [0.85 0.33 0.10]);
xlim([0 10000]);
xlabel('频率 (Hz)');
ylabel('幅度');
title('幅度谱（线性）—— 可见 750 Hz 的奇次谐波');
grid on;

% ---- 幅度谱（dB，0~20 kHz）----
subplot(3,1,3);
mag_dB = 20*log10(mag_pos + eps);    % 加 eps 防止 log(0)
plot(f_pos, mag_dB, 'LineWidth', 1.2, 'Color', [0.47 0.67 0.19]);
xlim([0 20000]);
ylim([-80 5]);
xlabel('频率 (Hz)');
ylabel('幅度 (dB)');
title('幅度谱（dB）—— 谐波以约 1/n 衰减 (-6 dB/oct)');
grid on;

%% 5. 打印前几个谐波的理论/实测频率
fprintf('基频 f0 = %.1f Hz\n', f0);
fprintf('理论奇次谐波频率: ');
fprintf('%.0f  ', f0*(1:2:11));
fprintf('\n');