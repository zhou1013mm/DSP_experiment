%% analyze_capture_fft.m
clear; clc; close all;

% ===== config =====
fs_full = 48000/4;        % ADC/DAC 实际采样率
decim   = 1;            % 必须和 C 代码 CAPTURE_DECIMATION 一致
fs      = fs_full / decim;

rawFile  = 'raw.dat';
procFile = 'processed.dat';

% ===== load =====
raw  = load(rawFile);
proc = load(procFile);

raw  = raw(:);
proc = proc(:);

if length(raw) ~= length(proc)
    warning('raw (%d) 和 proc (%d) 长度不一致，取前 %d 个', ...
            length(raw), length(proc), min(length(raw), length(proc)));
end
N = min(length(raw), length(proc));
raw  = double(raw(1:N));
proc = double(proc(1:N));
fprintf('Samples used: %d, fs = %g Hz\n', N, fs);

rawZ  = raw - mean(raw);
procZ = proc - mean(proc);

%% ===== 1) FFT magnitude compare =====
Nfft = 2^nextpow2(N);
w = hann(N);

Xr = fft(rawZ .* w, Nfft);
Xp = fft(procZ .* w, Nfft);

f = (0:Nfft/2)' * fs / Nfft;
Ar = abs(Xr(1:Nfft/2+1));
Ap = abs(Xp(1:Nfft/2+1));

% 用同一个参考归一化，保留增益差异
ref = max(Ar + eps);
Ar_dB = 20*log10(Ar / ref + eps);
Ap_dB = 20*log10(Ap / ref + eps);

h1 = figure('Name','FFT Compare in dB','Color','w');
plot(f, Ar_dB, 'b', 'LineWidth', 1); hold on;
plot(f, Ap_dB, 'r', 'LineWidth', 1);

grid on; xlim([0,fs/2]);
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB, ref = raw peak)');
title('FFT Magnitude Comparison');
legend('Raw','Processed','Location','best');

%% ===== 2) Welch PSD compare =====
winLen   = 4096;
noverlap = round(winLen*0.5);
nfft_psd = 8192;

[Pr, f_psd] = pwelch(rawZ,  hann(winLen), noverlap, nfft_psd, fs, 'onesided');
[Pp, ~    ] = pwelch(procZ, hann(winLen), noverlap, nfft_psd, fs, 'onesided');

Pr_dB = 10*log10(max(Pr, 1e-20));
Pp_dB = 10*log10(max(Pp, 1e-20));

h2 = figure('Name','PSD Compare','Color','w');
plot(f_psd, Pr_dB, 'b', 'LineWidth', 1); hold on;
plot(f_psd, Pp_dB, 'r', 'LineWidth', 1); 
grid on; xlim([0,fs/2]);
xlabel('Frequency (Hz)');
ylabel('PSD (dB/Hz)');
title('Welch Power Spectral Density Comparison');
legend('Raw','Processed','Location','best');

%% ===== 3) FFT Amplitude compare =====

h3 = figure('Name','FFT Compare','Color','w');
plot(f, Ar, 'b', 'LineWidth', 1); hold on;
plot(f, Ap, 'r', 'LineWidth', 1);

grid on; xlim([0,fs/2]);
xlabel('Frequency (Hz)');
ylabel('Amplitude (ref = raw peak)');
title('FFT Amplitude Comparison');
legend('Raw','Processed','Location','best');


%% test

fs = fs_full;
f1 = 1000; f2 = 2000; f3 = 6000;
t = linspace(0, 0.5, 0.5*fs);
a1 = sin(2*pi*f1*t);
a2 = sin(2*pi*f2*t);
a3 = sin(2*pi*f3*t);
a = a1+a2+a3;

% 时域
h4 = figure;
plot(t, a/3);% hold on; plot(t,(a1+a2)/2,'r'); hold on; plot(t,(a1),'yellow'); 
xlim([0, 0.01]);

% 频域
N = length(a);
freq = fft(a);
f = (0:N-1)*(fs/N);         % 频率轴

h5 = figure;
plot(f, abs(freq));
xlim([0, 10000]);           % 只看低频段，方便看 1000/2000/6000
xlabel('Frequency (Hz)'); ylabel('|FFT|');


% raw & proc , stride because data collection used int_16_t

stride = 1:1:24000;

raw_stride = raw(stride,1)./max(raw);
raw_stride = raw_stride - mean(raw_stride);
h6 = figure; plot(raw_stride');
% 频域
N = length(raw_stride);
freq_raw = fft(raw_stride');
f = (0:N-1)*(fs/N);         % 频率轴
% raw_stride freq test
h7 = figure;
plot(f,abs(freq_raw));
xlabel('Frequency (Hz)'); ylabel('|FFT|');

proc_stride = proc(stride,1)./max(proc);
proc_stride = proc_stride - mean(proc_stride);
h8 = figure; plot(proc_stride');
% 频域
N = length(proc_stride);
freq_proc = fft(proc_stride');
f = (0:N-1)*(fs/N);         % 频率轴
% proc_stride freq test
h9 = figure;
plot(f,abs(freq_proc));
xlabel('Frequency (Hz)'); ylabel('|FFT|');

fc = 2000;           % 截止频率
N_fir = 32;          % FIR 长度（31阶）

b_fir = fir1(N_fir-1, fc/(fs/2));            % FIR，31阶
h_fir = filter(b_fir, 1, raw_stride');
freq_matlab_proc = fft(h_fir);
h10 = figure; plot(f,abs(freq_matlab_proc));


%% 保存图片

saveas(h1, 'fft_compare_dB.png');
saveas(h2, 'psd_compare.png');
saveas(h3, 'fft_compare.png');
saveas(h4, 'matlab_sin.png');
saveas(h5, 'matlab_sin_freq.png');
saveas(h6, 'raw.png');
saveas(h7, 'raw_freq.png');
saveas(h8, 'proc.png');
saveas(h9, 'proc_freq.png');
saveas(h10,'matlab_proc_freq.png')