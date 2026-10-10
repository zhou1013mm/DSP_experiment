fs = 48000; T = 10; t = (0:fs*T-1)/fs;
% a = (sin(2*pi*1000*t) + sin(2*pi*2000*t)) / 2;
% audiowrite('synth_1k2k.wav', a, fs);
a = sin(2*pi*6000*t);
audiowrite('synth_6k.wav', a, fs);