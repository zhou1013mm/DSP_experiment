function [y] = AddConv(x,h,m)
%ADDCONV 这个函数用重叠相加法计算序列x通过H的响应
%   对x的每个分段取长度L,L=2^m+1-M,M是h(n)长度,将x的各个L长度的分段x_r(n)分别和h(n)
%   作圆周卷积，得到结果y_r(n),y_r(n)分别向右平移r*L后叠加得到y(n)
%   总共二个补零过程：1.将x(n)补零成能被L整除 2.将x各个分段xr，以及h补零成N=2^m=L+M-1长度
    M = length(h);
    N = 2^n;
    L = N+1-M;
    %这个判断有必要吗
    if M>L
        fprintf("2^m+1-M 应大于M");
        return
    end
    %补零，使x能被L整除，同时xr各分段和h再补零成N长度
    x = [x , zeros(1,L-mod(length(x),L))];
    xr = zeros(length(x)/L,N);
    h = [h,zeros(1,N-M)];
    Yr = zeros(length(x)/L,L+M-1);
    y = zeros(1,length(x)+M-1);
    H = fft(h);
    %将x分段为xr，然后FFT后乘上H得到Yr，
    %存储方式是每个分段作为Yr矩阵的一行
    for i = 1:length(x)/L
        xr(i,1:L) = x(1,(i-1)*L+1:i*L);
    end
    for i = 1:length(x)/L
        Yr(i,:) = H.*fft(xr(i,:));
    end
    %叠加法得到y
    for i = 1:length(x)/L
        y(1,(i-1)*L+1:i*L) = y(1,(i-1)*L+1:i*L)+ifft(Yr(i,:));
    end
end

