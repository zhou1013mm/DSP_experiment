
% a problem
N = 10;
x1 = [sin(pi*[0:1:5]./3),zeros(1,4)];
x2 = [cos(pi*[0:1:7]./2),zeros(1,2)];
ya = ifft(fft(x1).*fft(x2));
% ya1 = conv(x1,x2);
% figure; plot(ya1,'b');

% b problem
N = 32;
x1 = cos(2*pi*[0:1:N-1]/32);
x2 = sin(2*pi*[0:1:N-1]/32);
yb = ifft(fft(x1).*fft(x2));

% c problem
N = 20;
x1 = 0.8.^([0:1:N-1]);
x2 = (-0.8).^([0:1:N-1]);
yc = ifft(fft(x1).*fft(x2));

% d problem 
N = 10;
x1 = [0:1:N-1];
x2 = [N:-1:1];
yd = ifft(fft(x1).*fft(x2));

% e problem
N = 50;
x1 = [0.8.^[0:19],zeros(1,N-20)];
x2 = [ones(1,40),zeros(1,N-40)];
ye = ifft(fft(x1).*fft(x2));

% plot
figure("Position",[100,100,1400,300]);
subplot(151);plot(ya);xlabel('n');ylabel('y_a(n)');title("a");
subplot(152);plot(yb);xlabel('n');ylabel('y_b(n)');title("b");
subplot(153);plot(yc);xlabel('n');ylabel('y_c(n)');title("c");
subplot(154);plot(yd);xlabel('n');ylabel('y_d(n)');title("d");
subplot(155);plot(ye);xlabel('n');ylabel('y_e(n)');title("e");
