%% analyze_capture_fft.m
% 分析 48k -> 8k 降采样
% raw.dat      : 48kHz ADC 原始数据
% processed.dat: FIR 滤波 + 抽取 + ZOH 回 48kHz

clear; clc; close all;

fs_in  = 48000;
fs_out = 8000;
Q      = fs_in / fs_out;   % = 6

rawFile  = 'raw.dat';
procFile = 'processed.dat';

% ===== 读 hex =====
function v = read_int16_from_hex(fname)
    fid = fopen(fname, 'r');
    if fid < 0, error('cannot open %s', fname); end
    C = textscan(fid, '%s');
    fclose(fid);
    tok = regexprep(C{1}, '^0[xX]', '');
    b = uint8(hex2dec(tok));
    if mod(numel(b),2) ~= 0, b = b(1:end-1); end
    lo = double(b(1:2:end));
    hi = double(b(2:2:end));
    u16 = lo + 256*hi;
    v = u16;
    v(u16 >= 32768) = u16(u16 >= 32768) - 65536;
    v = v(:);
end

raw  = read_int16_from_hex(rawFile);
proc = read_int16_from_hex(procFile);
fprintf('raw: %d, proc: %d\n', numel(raw), numel(proc));

N = min(numel(raw), numel(proc));
raw  = double(raw(1:N));
proc = double(proc(1:N));
fprintf('Samples used: %d @ 48kHz (%.3f s)\n', N, N/fs_in);

%% ===== 1) 时域 @ 48kHz =====
t48 = (0:N-1)' / fs_in;

h0 = figure('Name','Time Domain','Color','w','Position',[100 100 900 600]);
subplot(3,1,1);
plot(t48*1000, raw, 'b'); grid on; xlim([0 5]);
xlabel('Time (ms)'); ylabel('Amp'); title('Raw (ADC Ch0) @ 48kHz');

subplot(3,1,2);
plot(t48*1000, proc, 'r'); grid on; xlim([0 5]);
xlabel('Time (ms)'); ylabel('Amp'); title('Processed @ 48kHz (ZOH 阶梯)');

% 8kHz 采样点
proc8 = proc(1:Q:end);
t8 = (0:numel(proc8)-1)' / fs_out;
subplot(3,1,3);
plot(t48*1000, proc, 'r'); hold on;
plot(t8*1000, proc8, 'ko', 'MarkerSize', 4, 'MarkerFaceColor','k');
grid on; xlim([0 5]);
xlabel('Time (ms)'); ylabel('Amp');
title('ZOH 与真实 8kHz 采样点 (黑点)');
legend('ZOH @48k','8kHz 采样点','Location','best');

%% ===== 2) 关键对比: 两条都降到 8kHz =====
raw8  = raw (1:Q:end);
proc8 = proc(1:Q:end);       % 已抽过 6, 与上面同一个变量
N8    = numel(proc8);
w8    = hann(N8);
Nfft8 = 2^nextpow2(N8);

Xr8 = fft((raw8  - mean(raw8 )) .* w8, Nfft8);
Xp8 = fft((proc8 - mean(proc8)) .* w8, Nfft8);
f8  = (0:Nfft8/2)' * fs_out / Nfft8;
Ar8 = abs(Xr8(1:Nfft8/2+1));
Ap8 = abs(Xp8(1:Nfft8/2+1));

ref8 = max(Ar8 + eps);

h1 = figure('Name','FFT @ 8kHz (正确对比)','Color','w','Position',[100 100 1000 500]);
plot(f8, 20*log10(Ar8/ref8 + eps), 'b', 'LineWidth', 1.2); hold on;
plot(f8, 20*log10(Ap8/ref8 + eps), 'r', 'LineWidth', 1.2);
grid on; xlim([0 fs_out/2]); ylim([-90 5]);
xlabel('Frequency (Hz)'); ylabel('dB (ref = raw8 peak)');
title('8kHz 采样下: Raw(未滤波抽取) vs Processed(滤波后抽取)');
legend('Raw (6k 混叠到 2k)','Processed (干净)','Location','best');
xline(1000,'k--'); xline(2000,'k--'); xline(3000,'k--');
text(1000,-85,'1k','HorizontalAlignment','center');
text(2000,-85,'2k (=6k 混叠位置)','HorizontalAlignment','center');
text(3000,-85,'3k','HorizontalAlignment','center');

%% ===== 2b) 8k 域频谱对比 (raw 直接抽 vs processed 滤波后抽) =====
raw8  = raw (1:Q:end);
proc8 = proc(1:Q:end);
N8    = numel(proc8);
w8    = hann(N8);
Nfft8 = 2^nextpow2(N8);

Xr8 = fft((raw8  - mean(raw8 )) .* w8, Nfft8);
Xp8 = fft((proc8 - mean(proc8)) .* w8, Nfft8);
f8  = (0:Nfft8/2)' * fs_out / Nfft8;
Ar8 = abs(Xr8(1:Nfft8/2+1));
Ap8 = abs(Xp8(1:Nfft8/2+1));

h1b = figure('Name','8k 域对比','Color','w','Position',[100 100 1000 500]);
plot(f8, 20*log10(Ar8/max(Ar8+eps) + eps), 'b', 'LineWidth', 1.2); hold on;
plot(f8, 20*log10(Ap8/max(Ar8+eps) + eps), 'r', 'LineWidth', 1.2);
grid on; xlim([0 fs_out/2]); ylim([-90 5]);
xlabel('Hz'); ylabel('dB');
title('8k 域: Raw 直接抽 vs Processed 抽 (含IIR低通)');
legend('Raw','Processed','Location','best');
xline(1000,'k--'); xline(1600,'g--'); xline(2000,'k--');
text(1000,-85,'1k'); text(1600,-85,'IIR fc'); text(2000,-85,'2k');
saveas(h1b, './assets/fft_8k_compare.png');

%% ===== 3) 48kHz FFT (解释你看到的 6k 从哪来) =====
Nfft = 2^nextpow2(N);
w    = hann(N);
Xr = fft((raw  - mean(raw )) .* w, Nfft);
Xp = fft((proc - mean(proc)) .* w, Nfft);
f48 = (0:Nfft/2)' * fs_in / Nfft;
Ar  = abs(Xr(1:Nfft/2+1));
Ap  = abs(Xp(1:Nfft/2+1));
ref = max(Ar + eps);

h2 = figure('Name','FFT @ 48kHz (含 ZOH 镜像)','Color','w','Position',[100 100 1000 500]);
plot(f48, 20*log10(Ar/ref + eps), 'b', 'LineWidth', 1); hold on;
plot(f48, 20*log10(Ap/ref + eps), 'r', 'LineWidth', 1);
grid on; xlim([0 12000]); ylim([-90 5]);
xlabel('Frequency (Hz)'); ylabel('dB (ref = raw peak)');
title('48kHz 下: processed 是降采样+IIR+上采样后的结果');
legend('Raw','Processed (ZOH)','Location','best');
xline(1000,'k--'); xline(2000,'k--'); xline(6000,'k--'); xline(10000,'k--');
text(1000,-85,'1k','HorizontalAlignment','center');
text(2000,-85,'2k','HorizontalAlignment','center');
text(6000,-85,'6k (2k 镜像)','HorizontalAlignment','center');
text(10000,-85,'10k (2k 镜像)','HorizontalAlignment','center');

%% ===== 4) Welch PSD =====
winLen   = min(4096, floor(N/4));
noverlap = round(winLen*0.5);
[Pr, fp] = pwelch(raw  - mean(raw ), hann(winLen), noverlap, 8192, fs_in);
[Pp, ~ ] = pwelch(proc - mean(proc), hann(winLen), noverlap, 8192, fs_in);

h4 = figure('Name','PSD','Color','w','Position',[100 100 1000 500]);
plot(fp, 10*log10(max(Pr,1e-20)), 'b'); hold on;
plot(fp, 10*log10(max(Pp,1e-20)), 'r'); grid on; xlim([0 12000]);
xlabel('Frequency (Hz)'); ylabel('PSD (dB/Hz)');
title('Welch PSD'); legend('Raw','Processed');

%% ===== 保存 =====
if ~exist('./assets','dir'), mkdir('./assets'); end
saveas(h0, './assets/time_domain.png');
saveas(h1, './assets/fft_8k_compare.png');
saveas(h2, './assets/fft_48k_zoh.png');
saveas(h4, './assets/psd.png');

fprintf('done.\n');