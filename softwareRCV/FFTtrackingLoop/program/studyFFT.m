close all;
clear all;
disp('-------------------------------------------');


a=[];
fc=2729; % center freq
fs=3e5;% sampling freq
startingtime = 0;    % 2pi cycle counter
datalen =  98+0.1;     % 2pi cycle counter
disp(sprintf('data length is %f cycles, %f second',datalen, datalen/fc));
disp(sprintf('freq resolution is %f Hz',fc/datalen));

timeaxis=[];
for kk=startingtime/fc:1/fs:(datalen+startingtime)/fc-1/fs
    timeaxis=[timeaxis kk]; %unit is second
end
a=-exp(j*2*pi*fc*timeaxis);
%a=cos(2*pi*fc*timeaxis);
ffta=fft(a);
p = angle(ffta);
% p = unwrap(angle(ffta));
f = (0:length(ffta)-1)'*fs/length(ffta);
point2bePlot = 2000000;
if (point2bePlot>length(a))
    point2bePlot = length(ffta);
end

figure;
plot(f,real(a),'*-')

figure;
subplot(2,1,1)
absffta=abs(ffta(1:point2bePlot));
plot(f(1:point2bePlot),absffta,'.-')
ylabel('Abs. Magnitude'), grid on
subplot(2,1,2)
plot(f(1:point2bePlot), p(1:point2bePlot)/pi,'.-'), grid on
ylabel('Phase [pi]'), grid on
xlabel('Frequency [Hertz]')
figure;
ifftffta=ifft(ffta);
plot(real(ifftffta),'*-');

index = find(absffta>(max(absffta)+min(absffta))/2);
%[temp,index] = max(absffta);
disp(sprintf('mag is %f     ',absffta(index)));
disp(sprintf('corresponding freq is %f Hz    ',(index-1)*fc/datalen));
disp(sprintf('phase is %f pi     ',p(index)/pi));
disp('-------------------------------------------');
