function [wrongXoccurTimes,Xratio,all_ccn,all_frr] = srch5ms(fs,fc,startingSample,MAGSIGN)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% see comments in Tsui P161 to P162
% This function accepts 6ms worthy of input data. use the first 1ms to get
% 1KHz frequency bin estimate, then use the 2nd 1ms data to get 200Hz frequency bin estimate, then use the 
% remaining 5ms worthy of data to calculate the phase change between
% consecutive 1ms data and get finer carrier frequency estimate.
% Input
%   fs: sampling rate(Hz)
%   fc: carrier center frequency(Hz)
%   startingSample: In the input data sequence, the starting index to be used
%   MAGSIGN: ipnut data, 1*5ms*5.714MHz
% Output
%   wrongXoccurTimes: it contain the occuring time of phase change between
%   continuous 1ms data violate the threshold. It is studied using 5ms
%   data, so the worst case when the frequency is not aligned,
%   corresponding value in the array will be 4. The array is of length 32,
%   each stands for one satellite.
%   Xratio: in the best 1KHz frequency bin, the ratio of correlation peak
%   over averaged correlation value over 1023 Code chips
%   all_ccn: length 32 vector, each element corresponds to the sampling
%   clock index where the correlation peak occurs
%   all_frr: length 32 vector, each element corresponds to the best
%   estimate of carrier frequency so far, after 1KHz bin, 200Hz bin, and
%   frequency adjustment by correlation magnitudes of the two adjacent
%   frequency points.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ConstantDefinition;

h=waitbar(0,'Pls wait');
wrongXoccurTimes=zeros(1,32);
Xratio=zeros(1,32);
all_ccn=[];
all_frr=[];
for kk=1:32
    if kk==18 % PRN18 is present with the strongest signal power
        tt=1;
    end
    waitbar(kk/32,h);
    svnum=kk;%input('enter satellite number=');
    msg=sprintf('start working with SV number %d',svnum);
    disp(msg);
    nn=[0:sampleNoIn1ms-1]; % index of sampling clock index within 1ms, starting with 0
    
    x2 = MAGSIGN(startingSample:6*sampleNoIn1ms+startingSample-1)'; % extract 6ms worthy of data from MAGSIGN
    yy=zeros(21,sampleNoIn1ms);
    code=digitizg(sampleNoIn1ms,fs,0,svnum); % generate local Code replica with length 1ms
    xf=fft(x2(1:sampleNoIn1ms)'); % FFT of 1ms input data
    for i=1:21 % search frequency bin with center freq -10K -9K -8K ...... 8K 9K 10K
        fr=fc-10000+(i-1)*1000;
        lc=code.*exp(j*2*pi*fr*ts*nn); % modulate local Code replica to carrier 
        lcf=fft(lc); % FFT of 1ms local replica
        yy(i,:)=ifft(xf.*conj(lcf)); % multiply the FFT result and IFFT the result
    end % in yy, 21 rows, each row contains the correlation result for one frequency bin
    [amp crw]=max(max(abs(yy'))); % index of the row    where the maximum occurs, corresponding to 1KHz frequency bin
    [amp ccn]=max(max(abs(yy)));  % index of the column where the maximum occurs
    pt_init=ccn; % set the initial search starting position (sampliing clock index)
    cfrq=fc+1000*(crw-11); % set the initial search starting frequency (Hz)
    
    z5=x2(pt_init:pt_init+5*sampleNoIn1ms-1); % extract the remaining 5 ms worthy of input data
    za5=z5'.*[code code code code code]; % wipe off the Code of this 5ms input data
    
    for i=1:5 % around the coarse 1KHz freq bin result to further zoom in to 200Hz freq bin
        fr=cfrq-400+(i-1)*200;
        mfrq0(i)=sum(za5(1:sampleNoIn1ms).*exp(j*2*pi*fr*ts*nn)); % wipe off the carrier of the 1ms input data with estimated Carrier
        mfrq1(i)=abs(mfrq0(i)); % non-coherent correlation total
    end % after this for loop, the non-coherent correlation total for each 200Hz bin is stored in array mfrq1
    [mamp mrw]=max(mfrq1);
    mfrq=cfrq+200*(mrw-3);
    fr=mfrq; % update fr with the best frequency estimate of 200Hz bin
    
    zb5=za5.*exp(j*2*pi*fr*ts*[0:5*sampleNoIn1ms-1]); % wipe off the carrier of the 5ms input data with newly estimated 200Hz bin Carrier
    zc5=diff(-angle(sum(reshape(zb5,sampleNoIn1ms,5)))); % average 5ms-inputs to be 5 complex correlation totals and get 4 phase changes
    zc5fix=zc5;
    threshold = 2.3*pi/5; % ??????????????????????????????????????????????????????????????????? 2.3*pi/5 = 80deg????
    
%     for i=1:4
%         if abs(zc5(i))>threshold
%             zc5(i)=zc5fix(i)-2*pi;
%             if abs(zc5(i))>threshold
%                 zc5(i)=zc5fix(i)+2*pi;
%                 if abs(zc5(i))>2.3*pi/5
%                     zc5(i)=zc5fix(i)-pi;
%                     if abs(zc5(i))>threshold
%                         zc5(i)=zc5fix(i)-3*pi;
%                         if abs(zc5(i))>threshold
%                             zc5(i)=zc5fix(i)+pi;
%                         end
%                     end
%                 end
%             end
%         end
%     end
    
    for in= 1 : 4 % 5ms data contains 4 phase changes between 1ms data
        for shiftpi = -2:2
            if abs(zc5(in))<threshold
                break;
            else
                zc5(in)=zc5fix(in)+shiftpi*pi;
                if abs(zc5(in))<threshold
                    break;
                end
            end
        end % try and see whether the phase change between two continuous 1ms data is below the given threshold, when it is adjust by -2 -1 1 or 2 pi. The reason behind is the phase could change from 0.1pi to 1.9pi by a -0.2pi phase change
        if (abs(zc5(in))>threshold)
            disp('it falls into 200 and 250Hz');
            wrongXoccurTimes(kk)=wrongXoccurTimes(kk)+1; % it contain the occuring time of phase change between continuous 1ms data violate the threshold
        end
    end

    dfrq=mean(zc5)*1000/(2*pi); % use the phase change value occured in 1ms time to derive the frequency update
    frr=fr+dfrq;
    Xratio(kk) = max(abs(yy(crw,1:sampleNoIn1ms)))/mean(abs(yy(crw,1:sampleNoIn1ms))); % in the best 1KHz frequency bin, the ratio of correlation peak over averaged correlation value over 1023 Code chips
%     figure;
%     plot(abs(yy(crw,1:sampleNoIn1ms)));
%     figure;
%     plot(abs(yy(:,ccn)),'*');
    format long g
    frr
    all_ccn = [all_ccn ccn];
    all_frr = [all_frr frr];
end
close(h);




% If SV exists, there won't be freq estimation falling into 200 and 250Hz.
% If SV doesn't exist, there could be freq estimation falling into 200 and 250Hz, and most likely there are.
% 
% 
% 
% start working with SV number 1
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% 
% frr =
% 
%           1401678.56362035
% 
% start working with SV number 2
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% 
% frr =
% 
%           1402446.87061361
% 
% start working with SV number 3
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% 
% frr =
% 
%           1413043.74705324
% 
% start working with SV number 4
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% 
% frr =
% 
%           1400260.52543665
% 
% start working with SV number 5
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% 
% frr =
% 
%            1403751.5348985
% 
% start working with SV number 6
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% 
% frr =
% 
%           1401604.01443021
% 
% start working with SV number 7
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% 
% frr =
% 
%           1412488.83065476
% 
% start working with SV number 8
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% 
% frr =
% 
%           1400103.48400122
% 
% start working with SV number 9
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% 
% frr =
% 
%           1403168.34665977
% 
% start working with SV number 10
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% 
% frr =
% 
%           1398034.91120452
% 
% start working with SV number 11
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% 
% frr =
% 
%           1405403.37641285
% 
% start working with SV number 12
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% 
% frr =
% 
%           1400494.10207432
% 
% start working with SV number 13
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% 
% frr =
% 
%           1398812.39450167
% 
% start working with SV number 14
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% 
% frr =
% 
%           1415668.30374177
% 
% start working with SV number 15	(SV exists)
% 
% frr =
% 
%           1401314.75925907
% 
% start working with SV number 16
% 
% frr =
% 
%           1402592.46906129
% 
% start working with SV number 17
% 
% frr =
% 
%           1408101.10050716
% 
% start working with SV number 18	(SV exists)
% 
% frr =
% 
%            1402073.9791848
% 
% start working with SV number 19
% 
% frr =
% 
%           1399541.46566697
% 
% start working with SV number 20
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% 
% frr =
% 
%           1414986.31171418
% 
% start working with SV number 21
% 
% frr =
% 
%           1403543.04938137
% 
% start working with SV number 22	(SV exists)
% 
% frr =
% 
%           1400498.57967511
% 
% start working with SV number 23
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% 
% frr =
% 
%            1409909.5016667
% 
% start working with SV number 24
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% 
% frr =
% 
%           1408422.67276141
% 
% start working with SV number 25
% 
% frr =
% 
%           1406611.80621389
% 
% start working with SV number 26	(SV exists)
% 
% frr =
% 
%           1406184.49943505
% 
% start working with SV number 27
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% 
% frr =
% 
%           1399559.81833825
% 
% start working with SV number 28
% 
% frr =
% 
%           1398450.34200114
% 
% start working with SV number 29	(SV exists)
% 
% frr =
% 
%           1405849.89787827
% 
% start working with SV number 30
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% 
% frr =
% 
%           1403545.51061431
% 
% start working with SV number 31
% 
% frr =
% 
%           1399069.80487577
% 
% start working with SV number 32
% it falls into 200 and 250Hz
% it falls into 200 and 250Hz
% 
% frr =
% 
%           1402512.55514977
% 










