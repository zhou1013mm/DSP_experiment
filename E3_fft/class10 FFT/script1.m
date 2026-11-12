n = 1:4096; f0=100; fs = 400; f = linspace(0,fs,length(n));
x = 10*sin(2*f0*pi*n/fs)+wgn(1,length(n),0);%别写成wgn(length(n),1,0)
X = fft(x,length(n));
my_X = my_fft(x);
figure("Position",[100 100 1200 600]);
subplot(311);plot(n/fs,x);xlabel("t/s");ylabel("x(t)");title("fs="+fs);axis tight;
subplot(312);plot(f,X);xlabel("f/Hz");ylabel("fft(x(t))");title("Matlab FFT");
subplot(313);plot(f,my_X);xlabel("f/Hz");ylabel("my_fft(x(t))");title("My FFT");