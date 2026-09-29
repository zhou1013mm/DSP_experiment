fs = 48000;          % 采样频率
fc = 2000;           % 截止频率
N_fir = 32;          % FIR 长度（31阶）
N_iir = 8;           % IIR 阶数（4级2阶）

% filter design
b_fir = fir1(N_fir-1, fc/(fs/2));            % FIR，31阶
[b_iir, a_iir] = butter(N_iir, fc/(fs/2));   % IIR，8阶

% impulse response
imp = [1, zeros(1, 127)];
h_fir = filter(b_fir, 1, imp);
h_iir = filter(b_iir, a_iir, imp);

% test using 4 sos
[sos, g] = tf2sos(b_iir,a_iir);
h_iir_test = imp;
for i = 1:4
    h_iir_test = filter(sos(i,1:3),sos(i,4:6),h_iir_test);    
end
h_iir_test = h_iir_test.*g;
h_iir_test - h_iir;
C_output = [0.000108, 0.001293, 0.007360, 0.026546, 0.068271, 0.133323, 0.204803, 0.251177, 0.243551, 0.175536, 0.070357, -0.030546, -0.089746, -0.092583, -0.051808, 0.002740, 0.041597, 0.049814, 0.031031, 0.001379, -0.021513, -0.027735, -0.018248, -0.001764, 0.011568, 0.015690, 0.010741, 0.001478, -0.006286, -0.008926, -0.006330, -0.001107, 0.003417, 0.005083, 0.003729, 0.000784, -0.001850, -0.002893, -0.002194, -0.000535, 0.000997, 0.001644, 0.001288, 0.000355, -0.000534, -0.000933, -0.000755, -0.000231, 0.000284, 0.000528, 0.000442, 0.000148, -0.000150, -0.000299, -0.000258, -0.000094, 0.000078, 0.000169, 0.000150, 0.000059, -0.000041, -0.000095, -0.000087, -0.000037, 0.000021, 0.000053, 0.000051, 0.000023, -0.000010, -0.000030, -0.000029, -0.000014, 0.000005, 0.000017, 0.000017, 0.000008, -0.000002, -0.000009, -0.000010, -0.000005, 0.000001, 0.000005, 0.000006, 0.000003, -0.000000, -0.000003, -0.000003, -0.000002, 0.000000, 0.000002, 0.000002, 0.000001, -0.000000, -0.000001, -0.000001, -0.000001, -0.000000, 0.000000, 0.000001, 0.000000, 0.000000, -0.000000, -0.000000, -0.000000, -0.000000, 0.000000, 0.000000, 0.000000, 0.000000, -0.000000, -0.000000, -0.000000, -0.000000, 0.000000, 0.000000, 0.000000, 0.000000, -0.000000, -0.000000, -0.000000, -0.000000, 0.000000, 0.000000, 0.000000, 0.000000, -0.000000, -0.000000, -0.000000];
figure; plot(h_iir);hold on ; plot(h_iir_test+0.05,"red","LineStyle","--");
hold on ; plot(C_output+0.1,"green","LineStyle",":");
legend("8 level iir","2x4 concatenated sos, + 0.05","C code output, + 0.1");

% 幅频、相频、群延迟
[H_fir, w] = freqz(b_fir, 1, 1024, fs);
[H_iir, ~] = freqz(b_iir, a_iir, 1024, fs);

% 群延迟（对相位求导）
gd_fir = -diff(unwrap(angle(H_fir))) / (2*pi*(fs/1024));
gd_iir = -diff(unwrap(angle(H_iir))) / (2*pi*(fs/1024));

%% 绘图对比
figure('Name','滤波器对比');

% 单位脉冲响应
subplot(2,2,1);
stem(0:127, h_fir, 'b.'); hold on;
stem(0:127, h_iir, 'r.'); hold off;
title('单位脉冲响应'); legend('FIR','IIR'); xlabel('n'); grid on;

% 幅频响应
subplot(2,2,2);
plot(w, 20*log10(abs(H_fir)), 'b'); hold on;
plot(w, 20*log10(abs(H_iir)), 'r'); hold off;
xline(fc,'k--','fc'); title('幅频响应'); legend('FIR','IIR');
xlabel('频率(Hz)'); ylabel('幅度(dB)'); ylim([-80 5]); grid on;

% 相频响应
subplot(2,2,3);
plot(w, unwrap(angle(H_fir)), 'b'); hold on;
plot(w, unwrap(angle(H_iir)), 'r'); hold off;
xline(fc,'k--','fc'); title('相频响应'); legend('FIR','IIR');
xlabel('频率(Hz)'); ylabel('相位(rad)'); grid on;

% 群延迟
subplot(2,2,4);
plot(w(1:end-1), gd_fir, 'b'); hold on;
plot(w(1:end-1), gd_iir, 'r'); hold off;
xline(fc,'k--','fc'); title('群延迟'); legend('FIR','IIR');
xlabel('频率(Hz)'); ylabel('样点'); ylim([0 40]); grid on;

%% 输入/输出信号对比
t = (0:255)/fs;
% 复合输入：低频(500Hz) + 高频(5kHz) + 通带边缘(2.5kHz)
x = sin(2*pi*500*t) + sin(2*pi*5000*t) + sin(2*pi*2500*t);

y_fir = filter(b_fir, 1, x);
y_iir = filter(b_iir, a_iir, x);

figure('Name','输入/输出信号');
subplot(4,1,1); plot(t, x);      title('输入信号'); grid on;
subplot(4,1,2); plot(t, y_fir);  title('FIR 输出'); grid on;
subplot(4,1,3); plot(t, y_iir);  title('IIR 输出'); grid on;
subplot(4,1,4);
plot(t, x, 'k:'); hold on;
plot(t, y_fir, 'b'); plot(t, y_iir, 'r'); hold off;
title('三者对比'); legend('输入','FIR','IIR'); grid on;