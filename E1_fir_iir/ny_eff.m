% display nyquist effect

h1 = figure;
subplot(4,1,1); plot(abs(fft(sin(1000*2*pi*(0:0.0001:1)))));title("fs = 10000Hz");axis tight;
subplot(4,1,2); plot(abs(fft(sin(1000*2*pi*(0:0.0002:1)))));title("fs = 5000Hz");axis tight;
subplot(4,1,3); plot(abs(fft(sin(1000*2*pi*(0:0.0005:1)))));title("fs = 2000Hz");axis tight;
subplot(4,1,4); plot(abs(fft(sin(1000*2*pi*(0:0.001:1)))));title("fs = 1000Hz");axis tight;
sgtitle("nyquist effect example on sin(2*\pi*fs*t) where fs = 1000Hz");
saveas(h1,"./assets/nq_eff.png");
