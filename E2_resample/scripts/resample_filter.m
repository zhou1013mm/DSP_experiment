% %% test
% t = (0:fs-1)/fs;   % 或 0:1/fs:1-1/fs
% x = sin(2*pi*f1*t);
% plot(t, x);        % 横轴应为时间
% xlim([0, 0.005]);   % 看几个周期就够了


%% 48kHz -> 8kHz 抗混叠低通滤波器设计 (Kaiser 窗)
% 用途: 降采样前低通滤波, 防止 4kHz 以上频率混叠到 8kHz 采样后的 0-4kHz
% 流程: x[48k] -> H(z)低通 -> 每6点抽1点 -> y[8k]

clear; clc; close all;

%% 1. 参数设置
fs_in   = 48000;            % 输入采样率
fs_out  = 8000;             % 输出采样率
Q       = fs_in / fs_out;   % 抽取因子 = 6
P = Q;

f_nyq_out = fs_out / 2;     % 输出 Nyquist = 4000 Hz

% 滤波器指标
fp  = 3600;                 % 通带边缘 (Hz) - 保留到 3.6 kHz
fst = 4000;                 % 阻带边缘 (Hz) - 4 kHz 以上必须衰减
Ap  = 1;                    % 通带纹波 (dB)
Ast = 80;                   % 阻带衰减 (dB)

fprintf('===== 滤波器设计参数 =====\n');
fprintf('输入采样率     : %d Hz\n', fs_in);
fprintf('输出采样率     : %d Hz\n', fs_out);
fprintf('抽取因子 Q     : %d\n', Q);
fprintf('通带边缘 fp    : %d Hz\n', fp);
fprintf('阻带边缘 fst   : %d Hz\n', fst);
fprintf('阻带衰减 Ast   : %d dB\n\n', Ast);

%% 2. 用 kaiserord 估计阶数和 beta
% kaiserord 中频率可指定以 Hz 为单位 (传入 fs)
f_vec = [fp, fst];
a_vec = [1, 0];                             % 通带=1, 阻带=0
dev   = [(10^(Ap/20)-1)/(10^(Ap/20)+1), 10^(-Ast/20)];

[n, Wn, beta, ftype] = kaiserord(f_vec, a_vec, dev, fs_in);

% 保证 n 为偶数 (滤波器长度为奇数 => Type I 线性相位)
n = n + mod(n, 2);

fprintf('===== Kaiser 估计结果 =====\n');
fprintf('估计阶数 n     : %d\n', n);
fprintf('滤波器长度 L   : %d\n', n+1);
fprintf('Kaiser beta    : %.4f\n', beta);
fprintf('群延迟 (样本)  : %.2f\n\n', n/2);

%% 3. 设计滤波器
fc      = (fp + fst) / 2;           % 截止频率取过渡带中点 = 3800 Hz
Wn_fir  = fc / (fs_in / 2);         % fir1 要求归一化到 fs_in/2

h = fir1(n, Wn_fir, 'low', kaiser(n+1, beta));

% 归一化直流增益为 1 (让通带幅度≈1)
h = h / sum(h);
h = h * P;   % 上采样用：补偿插零导致的 P 倍衰减

L = length(h);
fprintf('实际滤波器长度 : %d\n', L);
fprintf('实际群延迟     : %.2f 个输入样本\n\n', (L-1)/2);

%% 4. 幅频响应
[H, w] = freqz(h, 1, 16384, fs_in);

figure('Name', '滤波器频响', 'Position', [100 100 900 650]);

subplot(2,1,1);
plot(w, 20*log10(abs(H)+eps), 'b', 'LineWidth', 1.2); hold on;
yline(-Ast, 'r--', 'LineWidth', 1, 'Label', sprintf('阻带要求 -%d dB', Ast));
xline(f_nyq_out, 'k--', 'LineWidth', 1, 'Label', '4 kHz');
xline(fp,  'g--', 'LineWidth', 1, 'Label', sprintf('fp=%d', fp));
xline(fst, 'm--', 'LineWidth', 1, 'Label', sprintf('fst=%d', fst));
xlim([0 fs_in/2]); ylim([-130 5]);
grid on;
xlabel('频率 (Hz)'); ylabel('幅度 (dB)');
title('幅频响应 (全带 0-24 kHz)');

subplot(2,1,2);
plot(w, 20*log10(abs(H)+eps), 'b', 'LineWidth', 1.2); hold on;
xline(f_nyq_out, 'k--', 'LineWidth', 1);
xline(fp, 'g--', 'LineWidth', 1);
xline(fst, 'm--', 'LineWidth', 1);
xlim([0 8000]); ylim([-130 5]);
grid on;
xlabel('频率 (Hz)'); ylabel('幅度 (dB)');
title('幅频响应 (放大 0-8 kHz)');

%% 5. 通带细节
figure('Name', '通带细节', 'Position', [100 100 900 350]);
plot(w, 20*log10(abs(H)+eps), 'b', 'LineWidth', 1.2); hold on;
yline( Ap, 'r--'); yline(-Ap, 'r--');
xlim([0 fp]); ylim([-3 1]);
grid on;
xlabel('频率 (Hz)'); ylabel('幅度 (dB)');
title('通带 0 - 3.6 kHz 纹波');

%% 6. 脉冲响应与 Kaiser 窗
figure('Name', '脉冲响应与窗', 'Position', [100 100 900 500]);

subplot(2,1,1);
stem(0:L-1, h, '.', 'MarkerSize', 8);
grid on; xlabel('n'); ylabel('h[n]');
title('FIR 脉冲响应 (对称 => 线性相位)');

subplot(2,1,2);
w_k = kaiser(L, beta);
plot(0:L-1, w_k, 'r', 'LineWidth', 1.2);
grid on; xlabel('n'); ylabel('w[n]');
title(sprintf('Kaiser 窗 (\\beta = %.3f)', beta));

%% 7. 相位响应 (验证线性相位)
figure('Name', '相位响应', 'Position', [100 100 900 350]);
phi = unwrap(angle(H));
plot(w, phi * 180/pi, 'b', 'LineWidth', 1.2);
xlim([0 fs_in/2]); grid on;
xlabel('频率 (Hz)'); ylabel('相位 (度)');
title('相位响应 (应为直线 => 线性相位)');

%% 8. 群延迟
figure('Name', '群延迟', 'Position', [100 100 900 350]);
gd = grpdelay(h, 1, 16384, fs_in);
plot(w, gd, 'b', 'LineWidth', 1.2); hold on;
yline((L-1)/2, 'r--', 'LineWidth', 1, 'Label', sprintf('%.2f', (L-1)/2));
xlim([0 fp]); grid on;
xlabel('频率 (Hz)'); ylabel('群延迟 (样本)');
title('群延迟 (通带内应恒为 (L-1)/2)');

%% 9. 信号级验证: 多频正弦, 看混叠是否被压住
t = (0:4800-1) / fs_in;     % 0.1 秒

% 1 kHz (保留) + 3.5 kHz (保留) + 5 kHz (衰减) + 6 kHz (衰减)
% 若不滤波直接抽取: 5kHz -> 混叠到 3kHz, 6kHz -> 混叠到 2kHz
x = sin(2*pi*1000*t) + sin(2*pi*3500*t) + ...
    sin(2*pi*5000*t) + sin(2*pi*6000*t);

y_filt = filter(h, 1, x);       % FIR 滤波
y_out  = y_filt(1:Q:end);       % 每 Q=6 点抽 1 点

figure('Name', '信号级验证', 'Position', [100 100 900 650]);

subplot(3,1,1);
plot(t*1000, x, 'b'); xlim([0 30]);
grid on; xlabel('t (ms)'); ylabel('x');
title('原始信号 48kHz: 1k + 3.5k + 5k + 6k');

subplot(3,1,2);
plot(t*1000, y_filt, 'b'); xlim([0 30]);
grid on; xlabel('t (ms)'); ylabel('滤波后');
title('滤波后: 5k/6k 被衰减, 剩余 1k + 3.5k');

subplot(3,1,3);
t_out = (0:length(y_out)-1) / fs_out;
plot(t_out*1000, y_out, 'r'); xlim([0 30]);
grid on; xlabel('t (ms)'); ylabel('y');
title('抽取后 8kHz 输出');

% 频谱对比
figure('Name', '频谱对比', 'Position', [100 100 900 500]);

subplot(2,1,1);
[Px, Fx] = pwelch(x, [], [], 8192, fs_in);
plot(Fx/1000, 10*log10(Px), 'b'); grid on;
xlabel('频率 (kHz)'); ylabel('PSD (dB/Hz)');
title('原始信号频谱'); xlim([0 12]);

subplot(2,1,2);
[Py, Fy] = pwelch(y_out, [], [], 4096, fs_out);
plot(Fy/1000, 10*log10(Py), 'r'); grid on;
xlabel('频率 (kHz)'); ylabel('PSD (dB/Hz)');
title('抽取后 8kHz 信号频谱 (5k/6k 混叠分量被抑制)');
xlim([0 4]);

%% 10. 导出系数为 C 头文件
%filename = 'lpf_48k_to_8k.h';
filename = 'lpf_8k_to_48k.h';
fid = fopen(filename, 'w');

fprintf(fid, '/* ============================================\n');
fprintf(fid, ' * 48kHz -> 8kHz 抗混叠 FIR 低通滤波器\n');
fprintf(fid, ' * 设计方法 : Kaiser 窗\n');
fprintf(fid, ' * 阶数     : %d, 长度 : %d\n', n, L);
fprintf(fid, ' * 采样率   : %d Hz\n', fs_in);
fprintf(fid, ' * 通带边缘 : %d Hz\n', fp);
fprintf(fid, ' * 阻带边缘 : %d Hz\n', fst);
fprintf(fid, ' * 阻带衰减 : %d dB\n', Ast);
% fprintf(fid, ' * 抽取因子 : Q = %d\n', Q);
fprintf(fid, ' * 抽取因子 : P = %d\n', P);
fprintf(fid, ' * 群延迟   : %.4f 样本 (整数 %d)\n', (L-1)/2, round((L-1)/2));
fprintf(fid, ' * ============================================ */\n\n');

% fprintf(fid, '#ifndef LPF_48K_TO_8K_H\n#define LPF_48K_TO_8K_H\n\n');
fprintf(fid, '#ifndef LPF_8K_TO_48K_H\n#define LPF_8K_TO_48K_H\n\n');   % guard 改名
fprintf(fid, '#define LPF_LEN              %d\n', L);
fprintf(fid, '#define LPF_Q                %d\n', P);
fprintf(fid, '#define LPF_P                %d\n', P);
fprintf(fid, '#define LPF_GROUP_DELAY      %.4f\n', (L-1)/2);
fprintf(fid, '#define LPF_GROUP_DELAY_INT  %d\n\n', round((L-1)/2));

fprintf(fid, 'static const double lpf_coeff[LPF_LEN] = {\n');
for k = 1:L
    if mod(k-1, 4) == 0
        fprintf(fid, '    ');
    end
    fprintf(fid, '%.9e', h(k));
    if k < L
        fprintf(fid, ',');
    end
    if mod(k, 4) == 0
        fprintf(fid, '\n');
    else
        fprintf(fid, ' ');
    end
end
fprintf(fid, '\n};\n\n');
fprintf(fid, '#endif /* LPF_48K_TO_8K_H */\n');
fclose(fid);

fprintf('===== 完成 =====\n');
fprintf('系数已导出到: %s\n', filename);
fprintf('滤波器长度   : %d\n', L);
fprintf('群延迟       : %.2f 样本\n', (L-1)/2);