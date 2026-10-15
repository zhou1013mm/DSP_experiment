[b,a] = butter(8, 1600/(8000/2), 'low');
[sos,g] = tf2sos(b,a);
fprintf('g = %.6e\n', g);           % 看这个值
[H,w] = freqz(b, a, 1024, 8000);
fprintf('DC gain = %.6f\n', abs(H(1)));  % 应该 ≈ 1
