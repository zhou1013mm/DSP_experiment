

% 分析不同数据长度下的性能差距
lengths = 2.^(8:12);
speed_ratios = [];

for N = lengths
    x = randn(1, N) + 1i*randn(1, N);
    
    time_my = timeit(@() my_fft(x));
    time_matlab = timeit(@() fft(x));
    
    ratio = time_my / time_matlab;
    speed_ratios = [speed_ratios, ratio];
    fprintf('N=%d: 我的FFT/Matlab FFT = %.1fx\n', N, ratio);
end

figure;
plot(lengths, speed_ratios, 'ro-', 'LineWidth', 2);
xlabel('数据长度 N');ylabel('速度比 (我的/Matlab)');title('FFT性能差距分析');grid on;