function [X] = my_fft(x1)
%MY_FFT 自己编写的DSP课程 基-2 FFT 算法函数
%       算法方法：分时抽取法，按照ppt p22的结构
%
%       3 layer embedded loop: m-manipulation -> a-row as group -> b-col as knot
%       steps: 
%           1.reshape X  
%           2.later half of each group self update
%           3.cross transition by knot
    N = length(x1);
    M = log2(N);
    if(bitand(N,N-1)~=0)
        fprintf("N should be 2^n")
        exit;
    end
    X = x1 ;
    WN = exp(-j*2*pi/N);

    %Preprocessing: reshape the Matrix X -> 1 x 2^M to 2^M x 1
    for n = 1:M
        X = [X(:,1:2^(M-n));X(:,2^(M-n)+1:2^(M-n+1))];
    end

    % 3 layer embedded loop: m-manipulation -> a-row as group -> b-col as knot
    % steps: 
    %   1.reshape X  
    %   2.later half of each group self update
    %   3.cross transition by knot
    for m = 1: M
        %step1
        X_med = zeros(2^(M-m),2^m);
        for a = 1:2^(M-m)
            X_med(a,:) = [X(2*a-1,:),X(2*a,:)];
        end
        X = X_med;
        
        % loop by group
        for a = 1:2^(M-m)
            % loop by knot
            % step2
            for b = 1:2^(m-1)
                X(a,b+2^(m-1)) = X(a,b+2^(m-1))*WN^((b-1)*2^(M-m));
            end
            % step3
            for b = 1:2^(m-1)
                X(a,b)=X(a,b)+X(a,b+2^(m-1));
                X(a,b+2^(m-1)) = X(a,b) - 2*X(a,b+2^(m-1));
            end    
        end    
    end
end

