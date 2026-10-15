%% design_pipeline.m
% 设计 48k -> 8k -> (8k域IIR) -> 48k pipeline 的滤波器
% 导出为单个头文件 lpf_down_then_up.h

clear; clc; close all;

fs_in  = 48000;
fs_out = 8000;
P      = fs_in / fs_out;   % = 6

%% ===== 1. FIR 抗混叠低通 (48k -> 8k) =====
fp  = 3000;
fst = 4000;
Ap  = 1;
Ast = 50;

f_vec = [fp, fst];
a_vec = [1, 0];
dev   = [(10^(Ap/20)-1)/(10^(Ap/20)+1), 10^(-Ast/20)];
[n, ~, beta, ~] = kaiserord(f_vec, a_vec, dev, fs_in);
n = n + mod(n, 2);

fc = (fp + fst) / 2;
Wn = fc / (fs_in / 2);
h_dec = fir1(n, Wn, 'low', kaiser(n+1, beta));
h_dec = h_dec / sum(h_dec);       % 直流增益 = 1

L_fir  = length(h_dec);
gd_fir = (L_fir - 1) / 2;
fprintf('FIR: 阶=%d 长=%d 群延迟=%d\n', n, L_fir, round(gd_fir));

%% ===== 2. 上采样 FIR (同形状, 增益 ×P) =====
h_int = h_dec * P;

%% ===== 3. IIR 低通 (8k 域, 1.6kHz -3dB) =====
% 注意: 8k 采样下 2kHz 就是 Nyquist, 所以截止要略低, 1.6k~1.8k 都行
N_iir  = 8;
fc_iir = 1600;
Wn_iir = fc_iir / (fs_out / 2);
[b_iir, a_iir]  = butter(N_iir, Wn_iir, 'low');
[sos_iir, g_iir] = tf2sos(b_iir, a_iir);
n_sos = size(sos_iir, 1);
fprintf('IIR: 阶=%d SOS=%d g=%.6e\n', N_iir, n_sos, g_iir);

%% ===== 4. 画图验证 =====
figure('Name','Pipeline filter responses','Position',[100 100 900 650]);

subplot(2,1,1);
[H_dec, wd] = freqz(h_dec, 1, 4096, fs_in);
plot(wd, 20*log10(abs(H_dec)+eps), 'b', 'LineWidth', 1.2); grid on;
xlim([0 fs_in/2]); ylim([-100 5]);
xlabel('Hz'); ylabel('dB');
title(sprintf('48k FIR 抗混叠低通 (fp=%d, fst=%d, Ast=%d)', fp, fst, Ast));
xline(fp,'g--'); xline(fst,'m--'); xline(fc,'r--');

subplot(2,1,2);
[H_iir, wi] = freqz(b_iir, a_iir, 4096, fs_out);
plot(wi, 20*log10(abs(H_iir)+eps), 'b', 'LineWidth', 1.2); grid on;
xlim([0 fs_out/2]); ylim([-100 5]);
xlabel('Hz'); ylabel('dB');
title(sprintf('8k IIR 低通 (Butter %d 阶, fc=%dHz)', N_iir, fc_iir));
xline(fc_iir,'r--'); xline(2000,'k--');

%% ===== 5. 信号级验证 =====
t48 = (0:4800-1)/fs_in;
x   = sin(2*pi*1000*t48) + sin(2*pi*2000*t48) + sin(2*pi*6000*t48);

y48_filt = filter(h_dec, 1, x);
y8       = y48_filt(1:P:end);
y8_iir   = filter(b_iir, a_iir, y8);

% 手工上采样: 插零 -> 滤波 -> ×P
y_up_z   = zeros(1, length(y8_iir)*P);
y_up_z(1:P:end) = y8_iir;
y48_out  = filter(h_int, 1, y_up_z);

figure('Name','Pipeline signal check','Position',[100 100 900 700]);
subplot(3,1,1);
[Px,Fx] = pwelch(x,[],[],8192,fs_in);
plot(Fx/1000, 10*log10(Px+eps)); grid on;
xlim([0 12]); xlabel('kHz'); ylabel('dB/Hz');
title('原始 48k (1k + 2k + 6k)');

subplot(3,1,2);
[P8,F8] = pwelch(y8_iir,[],[],4096,fs_out);
plot(F8/1000, 10*log10(P8+eps)); grid on;
xlim([0 4]); xlabel('kHz'); ylabel('dB/Hz');
title('8k 域 (降采样 + IIR 后): 应只剩 1k + 2k');

subplot(3,1,3);
[P48,F48] = pwelch(y48_out,[],[],8192,fs_in);
plot(F48/1000, 10*log10(P48+eps)); grid on;
xlim([0 12]); xlabel('kHz'); ylabel('dB/Hz');
title('上采样回 48k: 1k/2k 保留, 6k 的镜像被 IIR 压掉');

%% ===== 6. 导出头文件 =====
fname = 'lpf_down_then_up.h';
fid = fopen(fname, 'w');

fprintf(fid, '/* ============================================\n');
fprintf(fid, ' * 48k -> 8k -> 48k pipeline 滤波器统一头文件\n');
fprintf(fid, ' * 由 MATLAB 生成\n');
fprintf(fid, ' *   1) FIR 抗混叠低通 (48k -> 8k), 系数总和 = 1\n');
fprintf(fid, ' *   2) IIR 低通       (8k 域, %.0fHz 截止)\n', fc_iir);
fprintf(fid, ' *   3) FIR 上采样     (8k -> 48k), 系数总和 = %d\n', P);
fprintf(fid, ' * ============================================ */\n\n');
fprintf(fid, '#ifndef LPF_DOWN_THEN_UP_H\n');
fprintf(fid, '#define LPF_DOWN_THEN_UP_H\n\n');

% --- 5.1 FIR 抗混叠 ---
fprintf(fid, '/* ---------- 48k -> 8k FIR 抗混叠 ---------- */\n');
fprintf(fid, '#define DEC_FIR_LEN              %d\n', L_fir);
fprintf(fid, '#define DEC_FIR_Q                %d\n', P);
fprintf(fid, '#define DEC_FIR_GROUP_DELAY_INT  %d\n\n', round(gd_fir));
fprintf(fid, 'static const float dec_fir_coeff[DEC_FIR_LEN] = {\n');
for k = 1:L_fir
    if mod(k-1, 4) == 0, fprintf(fid, '    '); end
    fprintf(fid, '%.9ef', h_dec(k));
    if k < L_fir, fprintf(fid, ','); end
    if mod(k, 4) == 0
        fprintf(fid, '\n');
    else
        fprintf(fid, ' ');
    end
end
fprintf(fid, '\n};\n\n');

% --- 5.2 FIR 上采样 ---
fprintf(fid, '/* ---------- 8k -> 48k FIR 上采样 ---------- */\n');
fprintf(fid, '#define INT_FIR_LEN              %d\n', L_fir);
fprintf(fid, '#define INT_FIR_P                %d\n', P);
fprintf(fid, '#define INT_FIR_GROUP_DELAY_INT  %d\n\n', round(gd_fir));
fprintf(fid, 'static const float int_fir_coeff[INT_FIR_LEN] = {\n');
for k = 1:L_fir
    if mod(k-1, 4) == 0, fprintf(fid, '    '); end
    fprintf(fid, '%.9ef', h_int(k));
    if k < L_fir, fprintf(fid, ','); end
    if mod(k, 4) == 0
        fprintf(fid, '\n');
    else
        fprintf(fid, ' ');
    end
end
fprintf(fid, '\n};\n\n');

% --- 5.3 IIR ---
fprintf(fid, '/* ---------- 8k 域 IIR 低通 ---------- */\n');
fprintf(fid, '#define LPF_IIR_N_SOS            %d\n', n_sos);
fprintf(fid, '#define LPF_IIR_GAIN             (%.9ef)\n\n', g_iir);
fprintf(fid, 'static const float lpf_iir_sos[LPF_IIR_N_SOS][6] = {\n');
for s = 1:n_sos
    fprintf(fid, '    { %.9ef, %.9ef, %.9ef, 1.0f, %.9ef, %.9ef }', ...
        sos_iir(s,1), sos_iir(s,2), sos_iir(s,3), sos_iir(s,5), sos_iir(s,6));
    if s < n_sos, fprintf(fid, ','); end
    fprintf(fid, '\n');
end
fprintf(fid, '};\n\n');

% --- 5.4 函数: 降采样 ---
fprintf(fid, '/* ============================================\n');
fprintf(fid, ' * 函数实现\n');
fprintf(fid, ' * ============================================ */\n\n');

fprintf(fid, '/* 48k -> 8k 抗混叠降采样 */\n');
fprintf(fid, 'static inline int downsample_48k_to_8k(const float *x, int n_in, float *y)\n');
fprintf(fid, '{\n');
fprintf(fid, '    int count = 0;\n');
fprintf(fid, '    int i, k;\n');
fprintf(fid, '    for (i = 0; i < n_in; i += DEC_FIR_Q) {\n');
fprintf(fid, '        float acc = 0.0f;\n');
fprintf(fid, '        for (k = 0; k < DEC_FIR_LEN; k++) {\n');
fprintf(fid, '            int idx = i - k;\n');
fprintf(fid, '            if (idx >= 0) acc += dec_fir_coeff[k] * x[idx];\n');
fprintf(fid, '        }\n');
fprintf(fid, '        y[count++] = acc;\n');
fprintf(fid, '    }\n');
fprintf(fid, '    return count;\n');
fprintf(fid, '}\n\n');

% --- 5.5 函数: IIR ---
fprintf(fid, '/* 8k 域 IIR 低通 (DF2T, 状态跨调用保持) */\n');
fprintf(fid, 'static inline void iir_lpf_8k(const float *x, float *y, int n)\n');
fprintf(fid, '{\n');
fprintf(fid, '    static float z[LPF_IIR_N_SOS][2] = {{0.0f}};\n');
fprintf(fid, '    int i, s;\n');
fprintf(fid, '    for (i = 0; i < n; i++) {\n');
fprintf(fid, '        float in = x[i];\n');
fprintf(fid, '        for (s = 0; s < LPF_IIR_N_SOS; s++) {\n');
fprintf(fid, '            const float *c = lpf_iir_sos[s];\n');
fprintf(fid, '            float out = c[0] * in + z[s][0];\n');
fprintf(fid, '            z[s][0] = c[1] * in - c[4] * out + z[s][1];\n');
fprintf(fid, '            z[s][1] = c[2] * in - c[5] * out;\n');
fprintf(fid, '            in = out;\n');
fprintf(fid, '        }\n');
fprintf(fid, '        y[i] = in * LPF_IIR_GAIN;\n');
fprintf(fid, '    }\n');
fprintf(fid, '}\n\n');

% --- 5.6 函数: 上采样 ---
fprintf(fid, '/* 8k -> 48k 多相上采样 */\n');
fprintf(fid, 'static inline int upsample_8k_to_48k(const float *x, int n_in, float *y)\n');
fprintf(fid, '{\n');
fprintf(fid, '    const int P = INT_FIR_P;\n');
fprintf(fid, '    const int branch_len = (INT_FIR_LEN + P - 1) / P;\n');
fprintf(fid, '    int n, m;\n');
fprintf(fid, '    for (n = 0; n < n_in * P; n++) {\n');
fprintf(fid, '        int phase = n %% P;\n');
fprintf(fid, '        int q     = n / P;\n');
fprintf(fid, '        float acc = 0.0f;\n');
fprintf(fid, '        for (m = 0; m < branch_len; m++) {\n');
fprintf(fid, '            int hidx = phase + m * P;\n');
fprintf(fid, '            if (hidx >= INT_FIR_LEN) break;\n');
fprintf(fid, '            int xidx = q - m;\n');
fprintf(fid, '            if (xidx >= 0 && xidx < n_in)\n');
fprintf(fid, '                acc += int_fir_coeff[hidx] * x[xidx];\n');
fprintf(fid, '        }\n');
fprintf(fid, '        y[n] = acc;\n');
fprintf(fid, '    }\n');
fprintf(fid, '    return n_in * P;\n');
fprintf(fid, '}\n\n');

fprintf(fid, '#endif /* LPF_DOWN_THEN_UP_H */\n');
fclose(fid);

fprintf('\n===== 完成 =====\n');
fprintf('导出: %s\n', fname);
fprintf('  DEC FIR: 长 %d, Q=%d, 群延迟 %d\n', L_fir, P, round(gd_fir));
fprintf('  INT FIR: 长 %d, P=%d, 增益 ×%d\n', L_fir, P, P);
fprintf('  IIR    : %d SOS, g=%.6e, fc=%d Hz\n', n_sos, g_iir, fc_iir);