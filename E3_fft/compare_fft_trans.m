%% ========================================================================
%  compare_fft_trans.m
%
%  对比「自写 my_fft」和「ADI 库函数 rfft32（trans.h）」：
%      1) 频谱结果是否一致
%      2) 运算耗时（周期数）谁快、快多少
%
%  ------------------------------------------------------------------------
%  数据来源：CCES 里 debug 跑一会儿后，把下面这些变量导出成 .dat 文件
%            （每个字节一行，十进制或 0x 十六进制都行），放在本脚本所在目录：
%
%      gFftSpec.dat        32 个复数 = 64 个 float（内存里是 re,im,re,im,...）
%      gRfftRe.dat         32 个 float   ┐ rfft32 的结果
%      gRfftIm.dat         32 个 float   ┘
%      gFftMag.dat         16 个 float   （|X[k]|/N，k = 0..15）
%      time_used.dat        2 个 float   （[1] my_fft，[2] rfft32，周期/次）
%      gFftBinHz.dat        1 个 float   （可选，谱线间隔；缺省按 8 kHz/32 算）
%      gFftFrameCount.dat   1 个 uint32  （可选，累计帧数）
%
%  导出方法（CCES）：Debug 暂停后，在 Expressions/变量窗口选中变量，
%  右键 -> Export / 或 Memory 窗口的 Save，格式选 “每行一个字节”。
%  第 1 个字节是 float 的最低字节（小端），本脚本按小端拼回 float。
%
%  说明：gFftMag 已经除过 N；gFftSpec / gRfft* 是未归一化的原始 FFT 值，
%        两者都来自同一帧数据、同样的 N 点、都不做归一化，可以直接比。
% ========================================================================

clear; clc; close all;

%% ---------------- 可调参数 ----------------
N          = 32;        % FFT 点数，与 my_fft.h 里的 MY_FFT_N 一致
CLK_HZ     = 450e6;     % SHARC+ 核时钟（SC589 EZ-Kit 默认 450 MHz），只影响 us 换算
DEFAULT_FS = 8000;      % 读不到 gFftBinHz.dat 时，按 8 kHz 采样算谱线间隔
                        % （E2 流水线打开时是 8 kHz；只看 E3 时是 48 kHz，此时建议导出 gFftBinHz）
TOL_REL    = 1e-4;      % 判定“结果一致”的相对误差门限（float32 + 两种算法路径）

fprintf('当前目录：%s\n\n', pwd);

%% ---------------- 读内存导出文件 ----------------
fprintf('读取数据文件：\n');
spec_b    = read_mem('gFftSpec.dat',      'single', 2*N);
rfft_re   = read_mem('gRfftRe.dat',       'single', N);
rfft_im   = read_mem('gRfftIm.dat',       'single', N);
mag_my    = read_mem('gFftMag.dat',       'single', N/2);
t_used    = read_mem('time_used.dat',     'single', 2);
bin_hz    = read_mem('gFftBinHz.dat',     'single', 1);
n_frames  = read_mem('gFftFrameCount.dat','uint32', 1);

% 谱线间隔
if isempty(bin_hz) || bin_hz <= 0
    bin_hz = DEFAULT_FS / N;
    fprintf('  （没有 gFftBinHz.dat，按 %.0f Hz 采样算：%.1f Hz/谱线）\n', DEFAULT_FS, bin_hz);
end

have_spec = numel(spec_b)   >= 2*N;
have_lib  = numel(rfft_re)  >= N && numel(rfft_im) >= N;
have_time = numel(t_used)   >= 2;

fprintf('\n');

%% ---------------- 1. 频谱对比 ----------------
k = (0:N/2-1).';            % 0 .. N/2-1
f = k * bin_hz;             % 对应频率 / Hz

if have_lib && (have_spec || numel(mag_my) >= N/2)

    % my_fft 的复数谱（从内存里 re,im 交错拼出来）
    if have_spec
        X_my = spec_b(1:2:end) + 1i*spec_b(2:2:end);
    else
        X_my = mag_my(:);   % 只有幅度时退化成幅度对比
    end

    % rfft32 的复数谱
    X_lib = rfft_re(:) + 1i*rfft_im(:);

    % 库函数只保证前 N/2+1 个点有效，这里只比 0..N/2-1
    Xm = X_my(1:N/2);
    Xl = X_lib(1:N/2);

    % --- 顺序自检：库函数输出是自然序还是位反序？-------------------
    idx_br = bin2dec(fliplr(dec2bin((0:N-1)', log2(N)))) + 1;   % 位反序置换
    mag_full_my  = abs(X_my(1:min(N,numel(X_my))));
    mag_full_lib = abs(X_lib);
    e_nat  = norm(mag_full_my - mag_full_lib(1:numel(mag_full_my)));
    e_br   = norm(mag_full_my - mag_full_lib(idx_br(1:numel(mag_full_my))));
    if e_br < 0.1*e_nat
        fprintf('【注意】rfft32 的输出看起来是位反序，已按位反序重排后再比较。\n');
        X_lib = X_lib(idx_br);
        Xl    = X_lib(1:N/2);
    end

    % --- 误差统计 -------------------------------------------------
    d_abs   = abs(Xm - Xl);
    scale   = max([abs(Xm); abs(Xl)]);
    if scale == 0, scale = 1; end
    d_rel   = d_abs / scale;

    % 两种实现的整体比例因子（正常应 ≈ 1，说明标度约定一致）
    denom = real(Xl') * real(Xl);
    if denom > 0
        k_scale = real(Xl') * real(Xm) / denom;
    else
        k_scale = NaN;
    end

    fprintf('===== 1. 频谱对比（N = %d，谱线间隔 %.1f Hz）=====\n', N, bin_hz);
    fprintf('  最大绝对误差      : %.3e\n', max(d_abs));
    fprintf('  最大相对误差      : %.3e  （以最大谱线为基准）\n', max(d_rel));
    fprintf('  幅度比例 X_my/X_lib: %.6f  （≈1 表示两边标度一致）\n', k_scale);
    if max(d_rel) < TOL_REL
        fprintf('  结论              : 两边频谱【一致】\n');
    else
        fprintf('  结论              : 两边频谱【有偏差】，请看下面的表和图\n');
    end

    % --- 逐谱线表格 ------------------------------------------------
    fprintf('\n  k   频率/Hz     |X_my|        |X_lib|        |误差|      相对误差\n');
    fprintf('  ---  ---------  ------------  ------------  ------------  ----------\n');
    for ii = 1:N/2
        fprintf('  %2d  %9.1f  %12.4f  %12.4f  %12.4e  %10.3e\n', ...
                k(ii), f(ii), abs(Xm(ii)), abs(Xl(ii)), d_abs(ii), d_rel(ii));
    end
    fprintf('\n');
else
    fprintf('===== 1. 频谱对比：跳过（缺少 gFftSpec.dat 或 gRfftRe/Im.dat）=====\n\n');
    Xm = []; Xl = []; d_abs = []; d_rel = [];
end

%% ---------------- 2. 耗时对比 ----------------
fprintf('===== 2. 运算耗时对比 =====\n');
if have_time
    t_my  = t_used(1);
    t_lib = t_used(2);
    us    = 1e6 / CLK_HZ;              % 1 个周期 = 多少 us

    fprintf('  my_fft : %8.1f 周期/次   (%7.3f us/次 @ %.0f MHz)\n', t_my,  t_my*us,  CLK_HZ/1e6);
    fprintf('  rfft32 : %8.1f 周期/次   (%7.3f us/次 @ %.0f MHz)\n', t_lib, t_lib*us, CLK_HZ/1e6);
    if t_lib > 0
        fprintf('  比值   : my_fft / rfft32 = %.2f 倍  ->  %s\n', ...
                t_my/t_lib, ternary_str(t_my > t_lib, 'my_fft 更慢', 'my_fft 更快'));
    end
    if ~isempty(n_frames)
        fprintf('  累计帧数: %d 帧（%.2f 秒；帧率 = 谱线间隔 %.1f Hz）\n', ...
                n_frames, n_frames/bin_hz, bin_hz);
        fprintf('  总周期  : my_fft %.3e，rfft32 %.3e\n', ...
                t_my*n_frames, t_lib*n_frames);
    end
else
    fprintf('  跳过（缺少 time_used.dat）\n');
    t_my = NaN; t_lib = NaN;
end
fprintf('\n');

%% ---------------- 3. 画图 ----------------
if ~isempty(Xm)
    figure('Name','my_fft vs rfft32','Position',[100 100 1100 700],'Color','w');

    % (1) 线性幅度谱
    subplot(2,2,1);
    stem(f/1000, abs(Xm), 'o-', 'LineWidth', 1.1); hold on;
    stem(f/1000, abs(Xl), 'x--', 'LineWidth', 1.1);
    grid on; xlabel('频率 / kHz'); ylabel('|X[k]|');
    legend('my\_fft','rfft32','Location','best');
    title('幅度谱（线性）');

    % (2) 归一化 dB 幅度谱
    subplot(2,2,2);
    ref = max(abs(Xm)); if ref == 0, ref = 1; end
    plot(f/1000, 20*log10(abs(Xm)/ref + eps), 'o-', 'LineWidth', 1.1); hold on;
    plot(f/1000, 20*log10(abs(Xl)/ref + eps), 'x--', 'LineWidth', 1.1);
    grid on; xlabel('频率 / kHz'); ylabel('dB（相对峰值）');
    ylim([-100 5]);
    legend('my\_fft','rfft32','Location','best');
    title('幅度谱（dB，各自归一化到峰值）');

    % (3) 误差
    subplot(2,2,3);
    semilogy(f/1000, max(d_abs, realmin), 'o-', 'LineWidth', 1.1);
    grid on; xlabel('频率 / kHz'); ylabel('|X_{my} - X_{lib}|');
    title('两边频谱的绝对误差');

    % (4) 耗时
    subplot(2,2,4);
    if have_time
        bar([t_my t_lib], 0.5, 'FaceColor', [0.3 0.6 0.9]);
        set(gca, 'XTickLabel', {'my\_fft','rfft32'});
        ylabel('周期 / 次');
        title(sprintf('耗时对比（my/rfft32 = %.2f）', t_my/t_lib));
        grid on;
        text(1, t_my,  sprintf(' %.0f', t_my),  'HorizontalAlignment','center', 'VerticalAlignment','bottom');
        text(2, t_lib, sprintf(' %.0f', t_lib), 'HorizontalAlignment','center', 'VerticalAlignment','bottom');
    else
        text(0.1, 0.5, '没有 time\_used.dat', 'Units','normalized');
        axis off;
    end

    try
        print(gcf, 'fft_vs_trans.png', '-dpng', '-r150');   % 用 print，图里不会带坐标区工具栏
        fprintf('图已保存：%s\\fft_vs_trans.png\n', pwd);
    catch
        saveas(gcf, 'fft_vs_trans.png');
        fprintf('图已保存：%s\\fft_vs_trans.png\n', pwd);
    end
end

%% ========================================================================
%                              本地函数
%% ========================================================================

function v = read_mem(fname, type, n_expect)
%READ_MEM 读取 CCES 导出的“每行一个字节”的内存文件，按小端拼成 type 类型
%   v = read_mem(fname, 'single'|'uint32', n_expect)
    v = [];
    if ~isfile(fname)
        fprintf('  [缺失] %-22s 跳过\n', fname);
        return;
    end
    b = read_bytes(fname);
    if isempty(b)
        fprintf('  [空]   %-22s 没解析到数字，跳过\n', fname);
        return;
    end
    switch lower(type)
        case 'single',              w = 4;
        case {'uint32','int32','single4'}, w = 4;
        case {'uint16','int16'},    w = 2;
        otherwise, error('read_mem: 不支持的类型 %s', type);
    end
    nbyte = numel(b);
    if mod(nbyte, w) ~= 0
        warning('%s 有 %d 个字节，不是 %d 的整数倍，尾部 %d 个字节忽略', ...
                fname, nbyte, w, mod(nbyte, w));
        b = b(1:floor(nbyte/w)*w);
    end
    v = double(typecast(uint8(b(:)), lower(type)));
    if nargin > 2 && ~isempty(n_expect) && numel(v) > n_expect
        v = v(1:n_expect);
    end
    if nargin > 2 && numel(v) < n_expect
        fprintf('  [注意] %-22s 只有 %d 个值，期望 %d 个\n', fname, numel(v), n_expect);
    end
    fprintf('  [OK]   %-22s %d 个字节 -> %d 个 %s\n', fname, nbyte, numel(v), type);
end

function b = read_bytes(fname)
%READ_BYTES 从文本里按顺序取出所有数字（支持 12 / 0x0C 两种写法），并折到 0..255
    txt = fileread(fname);
    tok = regexp(txt, '0[xX][0-9a-fA-F]+|[-+]?\d+', 'match');
    b = zeros(numel(tok), 1);
    for i = 1:numel(tok)
        s = tok{i};
        if numel(s) > 2 && (strcmpi(s(1:2), '0x'))
            b(i) = hex2dec(s(3:end));
        else
            b(i) = str2double(s);
        end
    end
    b = mod(b, 256);        % CCES 里若显示成有符号字节（-128..127），这里转回 0..255
    b = uint8(b);
end

function s = ternary_str(cond, a, b)
    if cond, s = a; else, s = b; end
end
