%% analyze_capture_fft.m
clear; clc; close all;

% ===== config =====
fs = 2000;                 % 你当前导出等效采样率 (48000/24)
rawFile  = 'raw.dat';
procFile = 'processed.dat';

% ===== load =====
raw  = load(rawFile);
proc = load(procFile);

raw  = raw(:);
proc = proc(:);

N = min(length(raw), length(proc));
raw  = double(raw(1:N));
proc = double(proc(1:N));

fprintf('Samples used: %d\n', N);

% 去均值，减少DC影响
rawZ  = raw  - mean(raw);
procZ = proc - mean(proc);

%% ===== 1) FFT magnitude compare =====
Nfft = 2^nextpow2(N);
w = hann(N); % 加窗减少泄漏

Xr = fft(rawZ .* w, Nfft);
Xp = fft(procZ .* w, Nfft);

f = (0:Nfft/2)' * fs / Nfft;

Ar = abs(Xr(1:Nfft/2+1));
Ap = abs(Xp(1:Nfft/2+1));

% 幅度转 dB（归一化到各自峰值，便于形状对比）
Ar_dB = 20*log10(Ar / max(Ar + eps) + eps);
Ap_dB = 20*log10(Ap / max(Ap + eps) + eps);

figure('Name','FFT Compare','Color','w');
plot(f, Ar_dB, 'b', 'LineWidth', 1); hold on;
plot(f, Ap_dB, 'r', 'LineWidth', 1);
grid on; xlim([0 fs/2]);
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB, normalized)');
title('FFT Magnitude Comparison');
legend('Raw','Processed','Location','best');

%% ===== 2) Welch PSD compare (更平滑，报告更好看) =====
win = 1024;
noverlap = round(win*0.5);
nfft_psd = 2048;

[Pr, f_psd] = pwelch(rawZ,  win, noverlap, nfft_psd, fs, 'onesided');
[Pp, ~    ] = pwelch(procZ, win, noverlap, nfft_psd, fs, 'onesided');

figure('Name','PSD Compare','Color','w');
plot(f_psd, 10*log10(Pr + eps), 'b', 'LineWidth', 1); hold on;
plot(f_psd, 10*log10(Pp + eps), 'r', 'LineWidth', 1);
grid on; xlim([0 fs/2]);
xlabel('Frequency (Hz)');
ylabel('PSD (dB/Hz)');
title('Welch Power Spectral Density Comparison');
legend('Raw','Processed','Location','best');

%% ===== 3) 频率响应近似（Processed/Raw） =====
H = Ap ./ (Ar + eps);
H_dB = 20*log10(H + eps);

figure('Name','Approx Transfer Function','Color','w');
plot(f, H_dB, 'k', 'LineWidth', 1);
grid on; xlim([0 fs/2]);
xlabel('Frequency (Hz)');
ylabel('Gain (dB)');
title('Approximate Transfer Function: Processed / Raw');

%% 可选保存图片
saveas(findobj('Name','FFT Compare'), 'fft_compare.png');
saveas(findobj('Name','PSD Compare'), 'psd_compare.png');
saveas(findobj('Name','Approx Transfer Function'), 'transfer_estimate.png');
