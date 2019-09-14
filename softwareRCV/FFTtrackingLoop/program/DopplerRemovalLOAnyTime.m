function cosPhi_rk = DopplerRemovalLOAnyTime( fractionalPhi0, DeltaOmega, SamplingInterval )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% fractionalPhi0:   a floating value, ranges from 0 to 2 \pi.
% DeltaOmega:       a K element vector, rad/s
% SamplingInterval: a floating value, second
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
datalength = length(DeltaOmega);
for i=2:datalength
    DeltaOmega(i)=DeltaOmega(i)+DeltaOmega(i-1);
end
cosPhi_rk = cos(fractionalPhi0 + DeltaOmega * SamplingInterval);