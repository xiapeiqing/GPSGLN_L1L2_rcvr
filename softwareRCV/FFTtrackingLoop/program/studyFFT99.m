clear all;
result = [];
VerticalindexTop2Down = [];
HorizontalindexLeft2right = [];
index1=0;
index2=0;
for startingtime=-0.5:0.1:0.5% 2pi cycle counter
    index1=index1+1;
    VerticalindexTop2Down(index1) = startingtime*2;
    index2 = 0;
    for datalen = 98-0.5-startingtime:0.1:98+0.5-startingtime% 2pi cycle counter
        index2=index2+1;
        tmp = mod(datalen+startingtime,1)*2;
        if tmp>=1
            tmp = tmp-2;
        end
        HorizontalindexLeft2right(index2) = tmp;
        startingtime
        close all;
        disp('-------------------------------------------');
        
        a=[];
        fc=1000; % center freq
        fs=3e5;% sampling freq
        disp(sprintf('data length is %f cycles, %f second',datalen, datalen/fc));
        disp(sprintf('freq resolution is %f Hz',fc/datalen));
        
        timeaxis=[];
        for kk=startingtime/fc:1/fs:(datalen+startingtime)/fc-1/fs
            timeaxis=[timeaxis kk]; %unit is second
        end
        a=exp(j*2*pi*fc*timeaxis);
        %a=sin(2*pi*fc*timeaxis);
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
        
        %index = find(absffta>(max(absffta)+min(absffta))/2);
        [temp,index] = max(absffta);
        disp(sprintf('mag is %f     ',absffta(index)));
        disp(sprintf('corresponding freq is %f Hz    ',(index-1)*fc/datalen));
        disp(sprintf('phase is %f pi     ',p(index)/pi));
        disp('-------------------------------------------');
        result(index1,index2) = p(index)/pi;
    end
end
[[NaN HorizontalindexLeft2right];[VerticalindexTop2Down' result]]
disp('first clomun top->down: starting phase, -pi to pi')
disp(VerticalindexTop2Down)
disp('left->right: end phase, -pi to pi')
disp(HorizontalindexLeft2right);

trimmedResult=[];
[L1,L2]=size(result);
for i1=1:L1
    for i2=1:L2
        if(result(i1,i2)>0.5)
            trimmedResult(i1,i2) = result(i1,i2) - 1;
        elseif(result(i1,i2)<-0.5)
            trimmedResult(i1,i2) = result(i1,i2) + 1;
        else
            trimmedResult(i1,i2) = result(i1,i2);
        end
    end
end
[[NaN HorizontalindexLeft2right];[VerticalindexTop2Down' trimmedResult]]
disp('first clomun top->down: starting phase, -pi to pi')
disp(VerticalindexTop2Down)
disp('left->right: end phase, -pi to pi')
disp(HorizontalindexLeft2right);
save studyfft99data