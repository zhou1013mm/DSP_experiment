x = [zeros(1,45),ones(1,10),zeros(1,45)];
h = [1,zeros(1,20),1,zeros(1,10)];
L = 5;
y = AddConv(x,h,L);
subplot(211);plot(1:100,x,'r');subplot(212);plot(1:131,y,'b');