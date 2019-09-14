function [newfcSV, asdf, codeShiftNum, codeShiftNumCurveFitting, UsedNoRarray] = trackingLoop(SVID,fcSV,z10ms)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% see Ch8 in Tsui P165
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SVID is the satellite PRN
% fcSV is the carrier center frequency estimated by the fine frequency estimation, see Ch7.13 in P150
% z10ms is the RF front end data stream lasting for 10 millisecond
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ConstantDefinition;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% section 1:
%   Carrier Tracking, see Ch8.6 in Tsui P173
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

code=digitizg(sampleNoIn1ms,fs,0,SVID); % generate PRN code according to the SVID
newKoverN = fcSV/fs; % see comment btw P178 & P179
newK = fcSV/1000; % use 1ms data to do fft, the freq resolution is 1K, so the desired fractional K will be fcSV/1000
localCW10ms = exp(-j*2*pi*newKoverN*[0:sampleNoIn1ms*trackingLoopPeriod-1]); % Generate the replica carrier wave

manyCode = [];
% suppose we have 10 ms RF front end data, the CA code generated is of 1ms
% duration, needed to be duplicated.
for pp=1:trackingLoopPeriod
    manyCode = [manyCode code];
end

% z10ms is the RF frontend output data
% z10ms'.*manyCode strips the CA code modulation, the result za10ms is a pure CW wave
za10ms=z10ms'.*manyCode;

% see Eq.8.40, P176, x(n)*e^{-j2\pi nk/N}
zb10ms=za10ms.*localCW10ms;

% see Eq.8.40, P176, realize the summation, resulting in 10 X(k), each X(k)
% represents fft result for 1ms data
% temppromptCode is a 1*10 vector
temppromptCode = sum(reshape(zb10ms,sampleNoIn1ms,trackingLoopPeriod));
% get the phase information for each 1ms data, in radians, ranges from
% [-pi, pi)
% So far, the discontinuity in kernel function has not been eliminated
temppromptCodeAngle = angle(temppromptCode);

% Compare it with localCW10ms, the following function is used to achieve
% the phase information at the beginning of each 1 ms data set. See first line in P180
% In fact it should be e^{-j*2*pi*newK*[0:trackingLoopPeriod]*N/N}, where
% [0:trackingLoopPeriod]*N is the time domain index of each point with
% phase discontinuity.
tempdiscontinuity=angle(exp(-j*2*pi*newK*[0:trackingLoopPeriod]));
discontinuity1ms = [];
for aa = 1:trackingLoopPeriod
    discontinuity1ms(aa) = tempdiscontinuity(aa+1) - tempdiscontinuity(aa);
end

%  See the 5th line in P180
Phi = (temppromptCodeAngle - discontinuity1ms)/pi; % 10 ms data length, Phi is of 10 elements, matching each 1ms CA code period. Unit is pi

DeltaPhi = diff(Phi);% unit is pi. 10 ms data length, DeltaPhi is of 9 elements 

% at the line below, we should see value near 0 or 2pi, which means tracking of pure CW.
% pi occurs every 20ms due to the navigation data modulation, it causes 180
% degree phase change
% The Phase change is only meaningful with in 0 to 2pi, whereas
% this requirement is fulfilled by previous frequency estimation which is
% fine enough. If the differential operation causes estimation of phase
% change greater than 2pi, or less than 0, shift it to 0 to 2pi.
DeltaPhi = mod(DeltaPhi, 2);        %unit is pi. unwrap 2pi cyclic period
asdf = DeltaPhi;
% screen out the 180 degree phase change if it exists, use the remaining
% data to get more precise estimation of carrier frequency 
indexofContinousTracking = find(abs(DeltaPhi-1)>0.5);   % unit is pi
DeltaPhi = DeltaPhi(indexofContinousTracking);          % unit is pi
% convert the phase near 2pi, eg, 1.9pi to be near 0, eg, -0.1pi
DeltaPhi(find(DeltaPhi>1)) = DeltaPhi(find(DeltaPhi>1)) - 2; % unit is pi
% we process 1ms data. Within 1ms, change of 2pi is equivalent to 1KHz.
% Here we have phase change \pm\pi, it is equivalent to \pm 500Hz.
if ~isempty(DeltaPhi)
    newfcSV = fcSV + 500*mean(DeltaPhi); % unit is Hz
%   newfcSV = fcSV + 500*mean(DeltaPhi)*2; according to the analysis to FFT
%   I think *2 is necessary. But in the similar processing in searching,
%   Tsui didn't put *2 in his implementation.
else
    newfcSV = fcSV;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% section 2:
%   Code Tracking
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

EarlyCode = cyclicShift(manyCode,1,ConstantD);% L1R2
za10msEarly=z10ms'.*EarlyCode;
zb10msEarly=za10msEarly.*localCW10ms;
tempEarly = sum(reshape(zb10msEarly,sampleNoIn1ms,trackingLoopPeriod));

LateCode = cyclicShift(manyCode,2,ConstantD);% L1R2
za10msLate=z10ms'.*LateCode;
zb10msLate=za10msLate.*localCW10ms;
tempLate = sum(reshape(zb10msLate,sampleNoIn1ms,trackingLoopPeriod));

yEarlyArray = [];
yLateArray = [];
for index = 1:trackingLoopPeriod
    yEarly = abs(tempEarly(index));
    yEarlyArray = [yEarlyArray; yEarly];
    yLate = abs(tempLate(index));
    yLateArray = [yLateArray; yLate];
end
Rarray = yLateArray./yEarlyArray;
[meaningfulR,meaningfulRIndex]=PickUpSimilar(Rarray,std(Rarray));
UsedNoRarray = length(meaningfulRIndex);
Raveraged = mean(meaningfulR);
d = ConstantD*ts*gold_rate;
x=(1-Raveraged)*(1-d)/(1+Raveraged)*2; % unit is chip width %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Big question here, why multiplying a constant>1???

codeShiftNum = round(x*fs/gold_rate);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% codeShiftNum=0;
% if Raveraged > RconstantMax
%     codeShiftNum = -1;
% elseif Raveraged < RconstantMin
%     codeShiftNum = 1;
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% section 3:
%   Code Tracking using Curve fitting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
EarlyCodenear = cyclicShift(manyCode,1,ConstantDnear);% L1R2
za10msEarlynear=z10ms'.*EarlyCodenear;
zb10msEarlynear=za10msEarlynear.*localCW10ms;
tempEarlynear = sum(reshape(zb10msEarlynear,sampleNoIn1ms,trackingLoopPeriod));

EarlyCodefar = cyclicShift(manyCode,1,ConstantDfar);% L1R2
za10msEarlyfar=z10ms'.*EarlyCodefar;
zb10msEarlyfar=za10msEarlyfar.*localCW10ms;
tempEarlyfar = sum(reshape(zb10msEarlyfar,sampleNoIn1ms,trackingLoopPeriod));

LateCodenear = cyclicShift(manyCode,2,ConstantDnear);% L1R2
za10msLatenear=z10ms'.*LateCodenear;
zb10msLatenear=za10msLatenear.*localCW10ms;
tempLatenear = sum(reshape(zb10msLatenear,sampleNoIn1ms,trackingLoopPeriod));

LateCodefar = cyclicShift(manyCode,2,ConstantDfar);% L1R2
za10msLatefar=z10ms'.*LateCodefar;
zb10msLatefar=za10msLatefar.*localCW10ms;
tempLatefar = sum(reshape(zb10msLatefar,sampleNoIn1ms,trackingLoopPeriod));

%temppromptCode

yEarlyArraynear = abs(tempEarlynear);
[temp,tempIndex]=PickUpSimilar(yEarlyArraynear,std(yEarlyArraynear));
yEarlyArraynearAveraged = mean(temp);

yLateArraynear = abs(tempLatenear);
[temp,tempIndex]=PickUpSimilar(yLateArraynear,std(yLateArraynear));
yLateArraynearAveraged = mean(temp);

yEarlyArrayfar = abs(tempEarlyfar);
[temp,tempIndex]=PickUpSimilar(yEarlyArrayfar,std(yEarlyArrayfar));
yEarlyArrayfarAveraged = mean(temp);

yLateArrayfar = abs(tempLatefar);
[temp,tempIndex]=PickUpSimilar(yLateArrayfar,std(yLateArrayfar));
yLateArrayfarAveraged = mean(temp);

yPromptArray = abs(temppromptCode);
[temp,tempIndex]=PickUpSimilar(yPromptArray,std(yPromptArray));
yPromptArrayAveraged = mean(temp);

y1y2y3y4y5=[yEarlyArrayfarAveraged yEarlyArraynearAveraged yPromptArrayAveraged yLateArraynearAveraged yLateArrayfarAveraged];
[max1to5, max1to5index] = max(y1y2y3y4y5);
if( max1to5index > 4 || max1to5index < 2 )
    disp('curve fitting error')
    codeShiftNumCurveFitting = 0;
else
    y1y2y3 = y1y2y3y4y5(max1to5index-1:max1to5index+1); % e,p,l
    X1 = -dnear;
    X2 = 0;
    X3 = dnear;
    Xmatrix = [X1^2 X1 1; X2^2 X2 1; X3^2 X3 1];
    Xinv = inv(Xmatrix);
    abc = Xinv*y1y2y3';
    
    x=-abc(2)/(2*abc(1));
    codeShiftNumCurveFitting = round(x*fs/gold_rate);
end










