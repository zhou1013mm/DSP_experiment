%% 多速率信号处理：混叠与镜像演示
% 对比降采样/升采样时，不加低通滤波 vs 加低通滤波
clear; clc; close all;

%% ========== 参数设置 ==========
fs_high = 48000;   % 原采样率 48 kHz
fs_low  = 8000;    % 降采样后采样率 8 kHz
M = fs_high / fs_low;   % 降采样因子 = 6
L = fs_high / fs_low;   % 升采样因子 = 6

% 信号参数：包含一个低频成分和一个高频成分
f_low  = 1000;     % 低频 1 kHz（希望保留）
f_high = 6000;     % 高频 6 kHz（降采样后会混叠）
A_low  = 1;
A_high = 0.8;

% 时间向量（原采样率）
T = 0.01;          % 10 ms
t = 0:1/fs_high:T-1/fs_high;
N = length(t);

% 构造输入信号
x = A_low*sin(2*pi*f_low*t) + A_high*sin(2*pi*f_high*t);

%% ========== 第一部分：降采样（Decimation） ==========
figure('Name','降采样：混叠演示','Position',[100 100 1200 800]);

% --- 1. 原始信号频谱 ---
subplot(3,2,1);
plot_spectrum(x, fs_high);
title('原始信号 x(n) 频谱（48 kHz）');
xlabel('频率 (Hz)'); ylabel('幅度');

% --- 2. 直接降采样（不加抗混叠滤波） ---
x_dec_noFilter = x(1:M:end);   % 每隔 M 个点取一个
subplot(3,2,2);
plot_spectrum(x_dec_noFilter, fs_low);
title('直接降采样（无抗混叠）→ 混叠');

% --- 3. 设计抗混叠低通滤波器 ---
% 截止频率取新奈奎斯特频率 4 kHz 附近
fc_aa = 3500;              % 截止频率 3.5 kHz
[b_aa, a_aa] = butter(6, fc_aa/(fs_high/2));  % 6阶巴特沃斯

% --- 4. 先滤波，再降采样 ---
x_filtered = filtfilt(b_aa, a_aa, x);   % 零相位滤波
x_dec_filtered = x_filtered(1:M:end);
subplot(3,2,3);
plot_spectrum(x_dec_filtered, fs_low);
title('抗混叠滤波后降采样 → 无混叠');

% --- 5. 时域对比 ---
subplot(3,2,4);
plot(t*1000, x, 'b'); hold on;
stem((0:length(x_dec_noFilter)-1)/fs_low*1000, x_dec_noFilter, 'r', 'filled');
legend('原始 48kHz', '直接降采样 8kHz');
title('时域：直接降采样'); xlabel('时间 (ms)'); ylabel('幅度');
xlim([0 5]);

subplot(3,2,5);
plot(t*1000, x, 'b'); hold on;
stem((0:length(x_dec_filtered)-1)/fs_low*1000, x_dec_filtered, 'g', 'filled');
legend('原始 48kHz', '抗混叠后降采样 8kHz');
title('时域：抗混叠后降采样'); xlabel('时间 (ms)'); ylabel('幅度');
xlim([0 5]);

% --- 6. 滤波器频响 ---
subplot(3,2,6);
[H, f] = freqz(b_aa, a_aa, 1024, fs_high);
plot(f, 20*log10(abs(H)));
title('抗混叠低通滤波器频响');
xlabel('频率 (Hz)'); ylabel('幅度 (dB)');
grid on; ylim([-80 5]);
xline(fs_low/2, 'r--', '新奈奎斯特 4kHz');

%% ========== 第二部分：升采样（Interpolation） ==========
figure('Name','升采样：镜像演示','Position',[150 150 1200 800]);

% 构造 8 kHz 的低采样率信号
t_low = 0:1/fs_low:T-1/fs_low;
x_low = A_low*sin(2*pi*f_low*t_low);   % 只保留 1 kHz 成分

% --- 1. 低采样率信号频谱 ---
subplot(3,2,1);
plot_spectrum(x_low, fs_low);
title('低采样率信号 y(n) 频谱（8 kHz）');

% --- 2. 直接补零升采样（不加抗镜像滤波） ---
x_up_noFilter = zeros(1, length(x_low)*L);
x_up_noFilter(1:L:end) = x_low;
subplot(3,2,2);
plot_spectrum(x_up_noFilter, fs_high);
title('补零升采样（无抗镜像）→ 镜像');

% --- 3. 设计抗镜像低通滤波器 ---
fc_ai = 3500;              % 截止频率 3.5 kHz
[b_ai, a_ai] = butter(6, fc_ai/(fs_high/2));  % 6阶巴特沃斯
% 注意：补零后信号能量下降 L 倍，滤波器需要增益 L
b_ai = b_ai * L;

% --- 4. 补零后滤波 ---
x_up_filtered = filtfilt(b_ai, a_ai, x_up_noFilter);
subplot(3,2,3);
plot_spectrum(x_up_filtered, fs_high);
title('抗镜像滤波后升采样 → 无镜像');

% --- 5. 时域对比 ---
subplot(3,2,4);
plot((0:length(x_up_noFilter)-1)/fs_high*1000, x_up_noFilter, 'r');
title('时域：补零升采样（无滤波）'); xlabel('时间 (ms)'); ylabel('幅度');
xlim([0 5]);

subplot(3,2,5);
plot((0:length(x_up_filtered)-1)/fs_high*1000, x_up_filtered, 'g'); hold on;
plot(t_low*1000, x_low, 'b--o', 'MarkerSize', 4);
legend('抗镜像后 48kHz', '原 8kHz 采样点');
title('时域：抗镜像后升采样'); xlabel('时间 (ms)'); ylabel('幅度');
xlim([0 5]);

% --- 6. 滤波器频响 ---
subplot(3,2,6);
[H, f] = freqz(b_ai, a_ai, 1024, fs_high);
plot(f, 20*log10(abs(H)));
title('抗镜像低通滤波器频响（含增益 L）');
xlabel('频率 (Hz)'); ylabel('幅度 (dB)');
grid on; ylim([-40 20]);
xline(fs_low/2, 'r--', '原奈奎斯特 4kHz');

%% ========== 辅助函数：绘制频谱 ==========
function plot_spectrum(x, fs)
    N = length(x);
    X = fftshift(fft(x));
    f = (-N/2:N/2-1) * (fs/N);
    plot(f, abs(X)/N, 'LineWidth', 1.2);
    grid on;
    xlabel('频率 (Hz)'); ylabel('幅度');
    xlim([-fs/2 fs/2]);
end