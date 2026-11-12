function [x2] = reverse(x1)
%REVERSE 翻转序列x1，返回等长序列x2
    N = length(x1);x2 = zeros(1,N);
    for i = 1:N
        x2(i) = x1(N-i+1); 
    end
end

