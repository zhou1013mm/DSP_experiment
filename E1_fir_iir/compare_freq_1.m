%% analyze_capture_fft.m
clear; clc; close all;

% ===== config =====
fs = 48000;                    % 每通道采样率 = LRCLK
rawFile  = 'raw.dat';
procFile = 'processed_fir.dat';
procFile2 = 'processed_iir.dat';
% rawFile  = 'raw_test.dat';
% procFile = 'processed_test.dat';

% ===== 读 hex 文件（每行一个 0xNN）=====
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
proc2 = read_int16_from_hex(procFile2);
fprintf('raw: %d, proc: %d\n', numel(raw), numel(proc));

N = min(numel(raw), numel(proc));
raw  = double(raw(1:N));
proc = double(proc(1:N));
fprintf('Samples used: %d, fs = %g Hz\n', N, fs);

rawZ  = raw  - mean(raw);
procZ = proc - mean(proc);
procZ2 = proc2 - mean(proc2);

%% ===== 1) 时域 =====
t = (0:N-1)'/fs;
h0 = figure('Name','Time Domain','Color','w');
subplot(3,1,1); plot(t*1000, raw); grid on;
xlabel('Time (ms)'); ylabel('Amp'); title('Raw (ADC Ch0)'); xlim([0 20]);
subplot(3,1,2); plot(t*1000, proc); grid on;
xlabel('Time (ms)'); ylabel('Amp'); title('Processed (FIR)'); xlim([0 20]);
subplot(3,1,3); plot(t*1000, proc2); grid on;
xlabel('Time (ms)'); ylabel('Amp'); title('Processed (IIR)'); xlim([0 20]);

%% ===== 2) FFT =====
Nfft = 2^nextpow2(N);
w = hann(N);
Xr = fft(rawZ .* w, Nfft);
Xp = fft(procZ .* w, Nfft);
Xp2 = fft(procZ2 .* w, Nfft);
f  = (0:Nfft/2)' * fs / Nfft;
Ar = abs(Xr(1:Nfft/2+1));
Ap = abs(Xp(1:Nfft/2+1));
Ap2 = abs(Xp2(1:Nfft/2+1));

ref = max(Ar + eps);
h1 = figure('Name','FFT dB','Color','w');
subplot(2,1,1);
plot(f, 20*log10(Ar/ref+eps), 'b', 'LineWidth', 1); hold on;
plot(f, 20*log10(Ap/ref+eps), 'r', 'LineWidth', 1);
grid on; xlim([0 10000]);
xlabel('Frequency (Hz)'); ylabel('dB (ref=raw peak)');
title('FFT Comparison, FIR'); legend('Raw','Processed','Location','best');
subplot(2,1,2);
plot(f, 20*log10(Ar/ref+eps), 'b', 'LineWidth', 1); hold on;
plot(f, 20*log10(Ap2/ref+eps), 'r', 'LineWidth', 1);
grid on; xlim([0 10000]);
xlabel('Frequency (Hz)'); ylabel('dB (ref=raw peak)');
title('FFT Comparison, IIR'); legend('Raw','Processed','Location','best');

h2 = figure('Name','FFT linear','Color','w');
subplot(2,1,1);
plot(f, Ar, 'b', 'LineWidth', 1); hold on;
plot(f, Ap, 'r', 'LineWidth', 1);
grid on; xlim([0 10000]);
xlabel('Frequency (Hz)'); ylabel('Amplitude');
title('FFT Amplitude, FIR'); legend('Raw','Processed','Location','best');
subplot(2,1,2);
plot(f, Ar, 'b', 'LineWidth', 1); hold on;
plot(f, Ap2, 'r', 'LineWidth', 1);
grid on; xlim([0 10000]);
xlabel('Frequency (Hz)'); ylabel('Amplitude');
title('FFT Amplitude, IIR'); legend('Raw','Processed','Location','best');

% %% ===== 3) 理想合成波对比 =====
% f1 = 1000; f2 = 2000; f3 = 6000;
% ideal = (sin(2*pi*f1*t) + sin(2*pi*f2*t) + sin(2*pi*f3*t)) / 3;
% Xi = fft((ideal - mean(ideal)) .* w, Nfft);
% Ai = abs(Xi(1:Nfft/2+1));
% 
% h3 = figure('Name','Ideal vs Measured','Color','w');
% subplot(2,1,1);
% plot(t*1000, ideal, 'k'); grid on; xlim([0 20]);
% xlabel('Time (ms)'); ylabel('Amp'); title('Ideal 1k+2k+6k');
% subplot(2,1,2);
% plot(f, Ar/max(Ar), 'b'); hold on;
% plot(f, Ai/max(Ai), 'k--');
% grid on; xlim([0 10000]);
% xlabel('Frequency (Hz)'); ylabel('Normalized');
% title('Raw FFT vs Ideal FFT'); legend('Raw','Ideal');

% %% ===== 4) Welch PSD =====
% winLen = min(4096, floor(N/4));
% noverlap = round(winLen*0.5);
% [Pr, fp] = pwelch(rawZ,  hann(winLen), noverlap, 8192, fs);
% [Pp, ~ ] = pwelch(procZ, hann(winLen), noverlap, 8192, fs);
% h4 = figure('Name','PSD','Color','w');
% plot(fp, 10*log10(max(Pr,1e-20)), 'b'); hold on;
% plot(fp, 10*log10(max(Pp,1e-20)), 'r'); grid on; xlim([0 10000]);
% xlabel('Frequency (Hz)'); ylabel('PSD (dB/Hz)');
% title('Welch PSD'); legend('Raw','Processed');

%% 保存
saveas(h0, './assets/time_domain.png');
saveas(h1, './assets/fft_dB.png');
saveas(h2, './assets/fft_linear.png');
% saveas(h3, './assets/ideal_vs_raw.png');
% saveas(h4, './assets/psd.png');


% %% test 大小端存储
% b = uint8(load('processed_test.dat'));   % 48000x1
% 
% % ---- Little-endian ----
% lo = double(b(1:2:end));
% hi = double(b(2:2:end));
% v_le = lo + 256*hi;
% v_le(v_le >= 32768) = v_le(v_le >= 32768) - 65536;
% 
% % ---- Big-endian ----
% hi2 = double(b(1:2:end));
% lo2 = double(b(2:2:end));
% v_be = lo2 + 256*hi2;
% v_be(v_be >= 32768) = v_be(v_be >= 32768) - 65536;
% 
% % 画出来看哪个像正弦
% figure;
% subplot(2,1,1); plot(v_le(1:200)); title('Little-endian 前200点');
% subplot(2,1,2); plot(v_be(1:200)); title('Big-endian 前200点');