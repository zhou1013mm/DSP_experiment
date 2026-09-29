fs = 48000;                     % 24000 点 / 0.5 秒

fid = fopen('raw.dat','r');
x = fread(fid, 'int16');        % 关键：指定 signed 类型
fclose(fid);

% 归一化到 [-1, 1]
x = double(x) / 32768;          % int16 的范围

sound(x, fs);                   % 直接播放
audiowrite('raw.wav', x, fs); % 或者存成 wav