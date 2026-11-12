%% ========================================================================
%  analyze_e4.m  ——  E4 重叠保留法 FIR：频谱 + 耗时 一次性分析
%
%  一次 debug、一次导出，就能得到实验报告要的全部图：
%      1) 时域：raw / 255 阶重叠保留 / 1023 阶 4 分区
%      2) 频谱：raw / 255 阶 / 1023 阶（Welch，标出 1k、2k、6k）
%      3) 滤波器幅频响应：255 阶 vs 1023 阶（fc = 2 kHz）
%      4) 耗时：重叠保留 vs 逐点 FIR 的 cycles/块，以及加速比
%
%  ------------------------------------------------------------------------
%  在 CCES 里 Debug 暂停后，从同一次运行导出下面 4 个变量
%  （右键变量 -> Export，格式选“每行一个字节”，小端）：
%
%      gRawCapture.dat              int16，CAPTURE_TOTAL_SAMPLES 个
%      gProcessedCapture.dat        int16，255 阶重叠保留输出
%      gProcessed1024Capture.dat    int16，1023 阶 4 分区输出
%      gE4Timing.dat                float，12 个（顺序见 .c 里的注释）
%
%  把这 4 个 .dat 和本脚本放在同一目录，直接运行即可。
%  图会保存成 E4_time.png / E4_spectrum.png /
%            E4_filter_response.png / E4_timing.png
%
%  说明：
%    - 本工程按 48 kHz 演示，FS 固定 48000；
%    - processed 比 raw 晚约 8 ms（255 阶）/ 16 ms（1023 阶），
%      时域图没有做对齐，频谱不受影响；
%    - 如果 gE4Timing.dat 里 cycle 全为 0，说明当前核不是 SHARC
%      （Blackfin 没有 cycle_count.h），脚本会跳过耗时柱状图。
% ========================================================================

clear; clc; close all;

%% ---------------- 可调参数 ----------------
FS          = 48000;        % 采样率
HOP         = 256;          % 帧移 / 每块输出点数
TAPS1       = 256;          % 255 阶
TAPS2       = 1024;         % 1023 阶
FC          = 2000;         % 截止频率
CLK_HZ      = 450e6;        % SHARC+ 核时钟，只用于 cycles -> us 换算
CAPTURE_TOTAL_SAMPLES = [];      % 空表示读文件里的全部点（250 ms = 12000 点）

fprintf('当前目录：%s\n\n', pwd);

%% ---------------- 读 CCES 导出的内存文件 ----------------
fprintf('读取数据文件：\n');
raw   = read_mem('gRawCapture.dat',           'int16',  CAPTURE_TOTAL_SAMPLES);
p256  = read_mem('gProcessedCapture.dat',     'int16',  CAPTURE_TOTAL_SAMPLES);
p1024 = read_mem('gProcessed1024Capture.dat', 'int16',  CAPTURE_TOTAL_SAMPLES);
tim   = read_mem('gE4Timing.dat',             'single', 12);

if isempty(raw) || isempty(p256)
    error(['缺少 gRawCapture.dat 或 gProcessedCapture.dat。' ...
           '请按 E3 的方式从 CCES 导出“每行一个字节”的 .dat 文件。']);
end
if isempty(p1024)
    warning('没有 gProcessed1024Capture.dat，1023 阶曲线将用 255 阶结果代替。');
    p1024 = p256;
end
if isempty(tim)
    warning('没有 gE4Timing.dat，耗时部分将跳过。');
    tim = zeros(12,1);
end
if numel(tim) < 12
    tim(numel(tim)+1:12) = 0;
end

% 三个采集数组长度取一致
n = min([numel(raw), numel(p256), numel(p1024)]);
raw   = double(raw(1:n));
p256  = double(p256(1:n));
p1024 = double(p1024(1:n));
t_ms  = (0:n-1).' / FS * 1000;

fprintf('  raw   : %d 点（%.1f ms）\n', n, n/FS*1000);
fprintf('  p256  : %d 点\n', numel(p256));
fprintf('  p1024 : %d 点\n', numel(p1024));
fprintf('  timing: %d 个 float\n\n', numel(tim));

%% ---------------- 1. 时域 ----------------
fig_time = figure('Name','E4 时域','Color','w','Position',[80 80 1100 650]);

subplot(2,1,1);
plot(t_ms, raw,   'k', 'LineWidth', 1.0); hold on;
plot(t_ms, p256,  'b', 'LineWidth', 1.0);
plot(t_ms, p1024, 'r', 'LineWidth', 1.0);
grid on; xlim([0 min(20, t_ms(end))]);
xlabel('时间 / ms'); ylabel('幅度 / LSB');
title('E4 时域（前 20 ms，未对齐延迟）');
legend('raw', '255 阶重叠保留', '1023 阶 4 分区', 'Location', 'best');

subplot(2,1,2);
plot(t_ms, raw,   'k', 'LineWidth', 0.8); hold on;
plot(t_ms, p256,  'b', 'LineWidth', 0.8);
plot(t_ms, p1024, 'r', 'LineWidth', 0.8);
grid on; xlim([0 t_ms(end)]);
xlabel('时间 / ms'); ylabel('幅度 / LSB');
title('E4 时域（整段采集）');
legend('raw', '255 阶重叠保留', '1023 阶 4 分区', 'Location', 'best');

print(fig_time, 'E4_time.png', '-dpng', '-r200');

%% ---------------- 2. 频谱（Welch） ----------------
[Praw,  F] = welch_psd(raw,   FS);
[P256,  ~] = welch_psd(p256,  FS);
[P1024, ~] = welch_psd(p1024, FS);

% 以 raw 的最大谱线为 0 dB，三条曲线用同一把尺子
p_ref = max(Praw);
if p_ref <= 0
    p_ref = 1;
end
Praw_db  = 10*log10(Praw  / p_ref);
P256_db  = 10*log10(P256  / p_ref);
P1024_db = 10*log10(P1024 / p_ref);

fig_spec = figure('Name','E4 频谱','Color','w','Position',[100 100 1100 600]);
plot(F, Praw_db,  'k', 'LineWidth', 1.0); hold on;
plot(F, P256_db,  'b', 'LineWidth', 1.0);
plot(F, P1024_db, 'r', 'LineWidth', 1.0);
grid on; xlim([0 24000]); ylim([-100 5]);
xlabel('频率 / Hz'); ylabel('相对幅度 / dB');
title('E4 频谱（Welch，raw 归一化到 0 dB）');
legend('raw', '255 阶重叠保留', '1023 阶 4 分区', 'Location', 'best');

yl = ylim;
for f0 = [1000 2000 6000]
    plot([f0 f0], yl, 'k--');
    text(f0 + 120, yl(1) + 8, sprintf('%d Hz', f0), 'FontSize', 9);
end

print(fig_spec, 'E4_spectrum.png', '-dpng', '-r200');

%% ---------------- 3. 滤波器幅频响应 ----------------
h256  = design_lpf(TAPS1, FC, FS);
h1024 = design_lpf(TAPS2, FC, FS);

[H256,  f_h] = freq_resp(h256,  FS);
[H1024, ~]   = freq_resp(h1024, FS);

fig_fir = figure('Name','E4 滤波器响应','Color','w','Position',[120 120 1100 600]);
plot(f_h, 20*log10(abs(H256)  + eps), 'b', 'LineWidth', 1.2); hold on;
plot(f_h, 20*log10(abs(H1024) + eps), 'r', 'LineWidth', 1.2);
grid on; xlim([0 24000]); ylim([-120 5]);
xlabel('频率 / Hz'); ylabel('幅度 / dB');
title(sprintf('E4 低通滤波器幅频响应（fc = %d Hz）', FC));
legend(sprintf('%d 阶（%d taps）', TAPS1-1, TAPS1), ...
       sprintf('%d 阶（%d taps）', TAPS2-1, TAPS2), 'Location', 'best');

yl = ylim;
for f0 = [FC 6000]
    plot([f0 f0], yl, 'k--');
    text(f0 + 120, yl(1) + 8, sprintf('%d Hz', f0), 'FontSize', 9);
end

print(fig_fir, 'E4_filter_response.png', '-dpng', '-r200');

%% ---------------- 4. 耗时对比 ----------------
t_os256   = tim(1);
t_dir256  = tim(2);
t_os1024  = tim(3);
t_dir1024 = tim(4);
spd256    = tim(5);
spd1024   = tim(6);
n_blocks  = tim(7);
err_rel256  = tim(8);
err_rel1024 = tim(9);
err_abs256  = tim(10);
err_abs1024 = tim(11);
fs_reported = tim(12);

have_cycles = any([t_os256 t_dir256 t_os1024 t_dir1024] > 0);

if have_cycles
    fig_timecost = figure('Name','E4 耗时','Color','w','Position',[140 140 900 560]);
    vals = [t_os256, t_dir256, t_os1024, t_dir1024];
    bar(vals);
    grid on;
    set(gca, 'XTickLabel', {'OS 255', 'Direct 255', 'OS 1023', 'Direct 1023'});
    ylabel('cycles / 块（256 点）');
    title(sprintf('E4 耗时对比：255 阶 %.2fx，1023 阶 %.2fx', spd256, spd1024));
    for k = 1:numel(vals)
        text(k, vals(k), sprintf('%.0f', vals(k)), ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    end
    print(fig_timecost, 'E4_timing.png', '-dpng', '-r200');
else
    fig_timecost = [];
    fprintf('  [提示] gE4Timing 里的 cycle 全为 0（非 SHARC 核？），跳过耗时图。\n');
end

%% ---------------- 5. 结果汇总（可直接抄进报告） ----------------
fprintf('\n================ E4 结果汇总 ================\n');
fprintf('采样率              : %.0f Hz\n', FS);
fprintf('块长 / 帧移         : %d 点（%.2f ms）\n', HOP, HOP/FS*1000);
fprintf('滤波器              : %d 阶 / %d 阶\n', TAPS1-1, TAPS2-1);
fprintf('处理块数            : %.0f\n', n_blocks);
if fs_reported > 0
    fprintf('固件报告的采样率    : %.0f Hz\n', fs_reported);
end

fprintf('\n--- 误差（重叠保留 vs 同阶逐点 FIR）---\n');
fprintf('255 阶 最大相对误差 : %.3e\n', err_rel256);
fprintf('255 阶 最大绝对误差 : %.1f LSB\n', err_abs256);
fprintf('1023 阶 最大相对误差: %.3e\n', err_rel1024);
fprintf('1023 阶 最大绝对误差: %.1f LSB\n', err_abs1024);

if have_cycles
    fprintf('\n--- 耗时（启动时 5 个测试块平均，每块 256 点）---\n');
    fprintf('                    cycles/块     cycles/点      us/块\n');
    fprintf('OS 255          %12.0f  %12.2f  %10.2f\n', ...
            t_os256,  t_os256/HOP,  t_os256/CLK_HZ*1e6);
    fprintf('Direct 255      %12.0f  %12.2f  %10.2f\n', ...
            t_dir256, t_dir256/HOP, t_dir256/CLK_HZ*1e6);
    fprintf('OS 1023         %12.0f  %12.2f  %10.2f\n', ...
            t_os1024, t_os1024/HOP, t_os1024/CLK_HZ*1e6);
    fprintf('Direct 1023     %12.0f  %12.2f  %10.2f\n', ...
            t_dir1024,t_dir1024/HOP,t_dir1024/CLK_HZ*1e6);
    fprintf('加速比 255      : %.2f x\n', spd256);
    fprintf('加速比 1023     : %.2f x\n', spd1024);
end

% 6 kHz 衰减（以 raw 为基准）
idx6 = find(F >= 6000, 1);
if ~isempty(idx6)
    fprintf('\n--- 6 kHz 衰减（相对 raw）---\n');
    fprintf('255 阶          : %.1f dB\n', ...
            10*log10((P256(idx6)  + eps) / (Praw(idx6) + eps)));
    fprintf('1023 阶         : %.1f dB\n', ...
            10*log10((P1024(idx6) + eps) / (Praw(idx6) + eps)));
end

fprintf('\n图已保存：E4_time.png / E4_spectrum.png / E4_filter_response.png');
if have_cycles
    fprintf(' / E4_timing.png\n');
else
    fprintf('\n');
end
fprintf('=============================================\n');

%% ========================================================================
%  局部函数
% ========================================================================

function [Pxx, F] = welch_psd(x, fs)
%WELCH_PSD 不依赖 Signal Processing Toolbox 的 Welch 功率谱
    x = double(x(:));
    x = x - mean(x);

    nfft = 2^floor(log2(max(512, numel(x)/2)));
    nfft = min(8192, nfft);
    if nfft < 64
        Pxx = zeros(2,1); F = [0; fs/2]; return;
    end

    noverlap = floor(nfft/2);
    step = nfft - noverlap;
    win = 0.5 - 0.5*cos(2*pi*(0:nfft-1)'/(nfft-1));

    nseg = floor((numel(x) - noverlap) / step);
    if nseg < 1
        nseg = 1;
    end

    P = zeros(nfft/2 + 1, 1);
    nused = 0;
    for k = 1:nseg
        idx = (k-1)*step + (1:nfft).';
        if idx(end) > numel(x)
            break;
        end
        seg = x(idx) .* win;
        X = fft(seg);
        P = P + abs(X(1:nfft/2 + 1)).^2;
        nused = nused + 1;
    end

    P = P / max(1, nused);
    P(2:end-1) = 2 * P(2:end-1);
    Pxx = P / (fs * sum(win.^2));
    F = (0:nfft/2)' * fs / nfft;
end

function h = design_lpf(taps, fc, fs)
%DESIGN_LPF 和 C 里 e4_design_lpf() 完全相同的 Hamming 窗低通
    M = taps - 1;
    n = (0:M)';
    x = 2*fc/fs * (n - M/2);
    s = ones(size(x));
    idx = abs(x) > 1e-12;
    s(idx) = sin(pi*x(idx)) ./ (pi*x(idx));
    w = 0.54 - 0.46*cos(2*pi*n/M);
    h = s .* w;
    h = h / sum(h);
end

function [H, f] = freq_resp(h, fs)
%FREQ_RESP 零填充 FFT 得到幅频响应（不需要 freqz）
    N = 16384;
    H = fft(h(:), N);
    H = H(1:N/2 + 1);
    f = (0:N/2)' * fs / N;
end

function v = read_mem(fname, type, n_expect)
%READ_MEM 读 CCES 导出的“每行一个字节”文本，按小端拼回数值
    v = [];
    if ~isfile(fname)
        fprintf('  [缺失] %-28s\n', fname);
        return;
    end

    txt = fileread(fname);
    tok = regexp(txt, '0[xX][0-9a-fA-F]+|[-+]?\d+', 'match');
    if isempty(tok)
        fprintf('  [空]   %-28s\n', fname);
        return;
    end

    b = zeros(numel(tok), 1);
    for i = 1:numel(tok)
        s = tok{i};
        if numel(s) > 2 && strcmpi(s(1:2), '0x')
            b(i) = hex2dec(s(3:end));
        else
            b(i) = str2double(s);
        end
    end
    b = uint8(mod(b, 256));

    switch lower(type)
        case 'single'
            w = 4;
        case {'uint32', 'int32'}
            w = 4;
        case {'uint16', 'int16'}
            w = 2;
        otherwise
            error('read_mem: 不支持的类型 %s', type);
    end

    nbyte = numel(b);
    if mod(nbyte, w) ~= 0
        warning('%s 有 %d 个字节，不是 %d 的整数倍，尾部忽略', ...
                fname, nbyte, w);
        b = b(1:floor(nbyte/w)*w);
    end

    v = double(typecast(b(:), lower(type)));
    if nargin > 2 && ~isempty(n_expect) && numel(v) > n_expect
        v = v(1:n_expect);
    end
    fprintf('  [OK]   %-28s %d 字节 -> %d 个 %s\n', ...
            fname, nbyte, numel(v), type);
end
