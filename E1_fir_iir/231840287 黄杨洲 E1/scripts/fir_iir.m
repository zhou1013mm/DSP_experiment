fs = 48000;          % 采样频率
fc = 2000;           % 截止频率
N_fir = 32;          % FIR 长度（31阶）
N_iir = 8;           % IIR 阶数（4级2阶）

% filter design
b_fir = fir1(N_fir-1, fc/(fs/2));            % FIR，31阶
b_fir_test = fir1(N_fir-1, fc/(16000/2));            % FIR，31阶 test fs = 16000Hz
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
C_output = [0.000000, 0.000001, 0.000005, 0.000023, 0.000085, 0.000247, 0.000610, 0.001320, 0.002563, 0.004545, 0.007474, 0.011517, 0.016776, 0.023257, 0.030855, 0.039342, 0.048378, 0.057532, 0.066306, 0.074176, 0.080635, 0.085229, 0.087598, 0.087504, 0.084849, 0.079688, 0.072223, 0.062789, 0.051832, 0.039881, 0.027506, 0.015286, 0.003775, -0.006539, -0.015253, -0.022073, -0.026826, -0.029464, -0.030056, -0.028782, -0.025907, -0.021768, -0.016743, -0.011225, -0.005602, -0.000227, 0.004593, 0.008615, 0.011670, 0.013665, 0.014583, 0.014473, 0.013446, 0.011661, 0.009306, 0.006590, 0.003721, 0.000900, -0.001698, -0.003927, -0.005681, -0.006893, -0.007539, -0.007632, -0.007221, -0.006382, -0.005210, -0.003815, -0.002309, -0.000799, 0.000615, 0.001852, 0.002849, 0.003563, 0.003977, 0.004093, 0.003931, 0.003529, 0.002937, 0.002212, 0.001413, 0.000599, -0.000176, -0.000864, -0.001430, -0.001848, -0.002106, -0.002202, -0.002147, -0.001957, -0.001658, -0.001280, -0.000856, -0.000417, 0.000007, 0.000390, 0.000711, 0.000954, 0.001113, 0.001183, 0.001171, 0.001083, 0.000934, 0.000738, 0.000513, 0.000277, 0.000045, -0.000168, -0.000349, -0.000490, -0.000585, -0.000634, -0.000637, -0.000598, -0.000524, -0.000423, -0.000304, -0.000177, -0.000051, 0.000066, 0.000168, 0.000249, 0.000306, 0.000338, 0.000345, 0.000329, 0.000293, 0.000241];
h0 = figure; plot(h_iir);hold on ; plot(h_iir_test+0.05,"red","LineStyle","--");
hold on ; plot(C_output+0.1,"green","LineStyle",":");
legend("8 level iir","2x4 concatenated sos, + 0.05","C code output, + 0.1");

% 幅频、相频、群延迟
[H_fir, w] = freqz(b_fir, 1, 1024, fs);
[H_iir, ~] = freqz(b_iir, a_iir, 1024, fs);
[H_fir_test, ~] = freqz(b_fir_test, 1, 1024, fs);

% 群延迟（对相位求导）
gd_fir = -diff(unwrap(angle(H_fir))) / (2*pi*(fs/1024));
gd_iir = -diff(unwrap(angle(H_iir))) / (2*pi*(fs/1024));

%% 绘图对比
h1 = figure('Name','滤波器对比','Position',[0,0,1000,800]);

% 单位脉冲响应
subplot(2,2,1);
stem(0:127, h_fir, 'b.'); hold on;
stem(0:127, h_iir, 'r.'); hold off;
title('单位脉冲响应'); legend('FIR','IIR'); xlabel('n'); grid on;

% 幅频响应
subplot(2,2,2);
plot(w, 20*log10(abs(H_fir)), 'b'); hold on;
plot(w, 20*log10(abs(H_fir_test)), 'green'); hold on;
plot(w, 20*log10(abs(H_iir)), 'r'); hold off;
xline(fc,'k--','fc'); title('幅频响应'); legend('FIR,fs=48000Hz','FIR,fs=16000Hz','IIR,fs=16000Hz');
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
xlabel('频率(Hz)'); ylabel('样点'); grid on;

% %% 输入/输出信号对比
% t = (0:255)/fs;
% % 复合输入：低频(500Hz) + 高频(5kHz) + 通带边缘(2.5kHz)
% x = sin(2*pi*500*t) + sin(2*pi*5000*t) + sin(2*pi*2500*t);
% 
% y_fir = filter(b_fir, 1, x);
% y_iir = filter(b_iir, a_iir, x);
% 
% h2 = figure('Name','输入/输出信号');
% subplot(4,1,1); plot(t, x);      title('输入信号'); grid on; axis tight;
% subplot(4,1,2); plot(t, y_fir);  title('FIR 输出'); grid on; axis tight;
% subplot(4,1,3); plot(t, y_iir);  title('IIR 输出'); grid on; axis tight;
% subplot(4,1,4);
% plot(t, x, 'k:'); hold on;
% plot(t, y_fir, 'b'); plot(t, y_iir, 'r'); axis tight;hold off;
% title('三者对比'); legend('输入','FIR','IIR'); grid on;

%% save pic

saveas(h0,"assets/imp_compare.png")
saveas(h1,"assets/filter_freq.png");
