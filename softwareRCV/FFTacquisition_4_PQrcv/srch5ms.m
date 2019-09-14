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
for kk=25:25 % HW emulator generates PRN1
    if kk==15
        tt=1;
    end
    waitbar(kk/32,h);
    svnum=kk;%input('enter satellite number=');
    msg=sprintf('start working with SV number %d',svnum);
    disp(msg);
    nn=[0:sampleNoIn1ms-1]; % index of sampling clock index within 1ms, starting with 0
    
    x2 = MAGSIGN'; % extract 1ms worthy of data from MAGSIGN
    yy=zeros(21,sampleNoIn1ms);
    code=digitizg(sampleNoIn1ms,fs,0,svnum); % generate local Code replica with length 1ms
    xf=fft(x2(1:sampleNoIn1ms)'); % FFT of 1ms input data
    inst_fr = [];
    for i=1:21 % search frequency bin with center freq -100 -99 -98 ...... 98 99 100
        fr=fc-1000+(i-1)*100;
        lc=code.*exp(j*2*pi*fr*ts*nn); % modulate local Code replica to carrier 
        lcf=fft(lc); % FFT of 1ms local replica
        yy(i,:)=ifft(xf.*conj(lcf)); % multiply the FFT result and IFFT the result
        inst_fr = [inst_fr fr];
    end % in yy, 21 rows, each row contains the correlation result for one frequency bin
    [amp crw]=max(max(abs(yy'))); % index of the row    where the maximum occurs, corresponding to 1KHz frequency bin
    [amp ccn]=max(max(abs(yy)));  % index of the column where the maximum occurs
    pt_init=ccn; % set the initial search starting position (sampliing clock index)
    cfrq=fc+1000*(crw-11); % set the initial search starting frequency (Hz)
    aa;
end 
    
    
