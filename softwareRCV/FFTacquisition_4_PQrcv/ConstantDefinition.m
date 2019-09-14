% ConstantDefinition.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fixed Constant Definition
fs=25e6; % sampling rate
fc=2.42e6; % carrier center frequency
sampleNoIn1ms=fs/1000;
ts=1/fs;
gold_rate=1.023e6;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Adjustable Constant Definition
trackingLoopPeriod = 10; % ms ?????
HowManyLoopClosure = 8; % ?????
ConstantD = 3; % 500ns/175ns=2.86\approx 3 ?????
% Tsui, 185
% x = gold_rate*ts/2;
% d = gold_rate*ConstantD*ts;
% d = 0.5371
% r = (1-x-d)/(1+x-d)
% % = 0.6759
% x = -gold_rate*ts/2;
% d = gold_rate*ConstantD*ts;
% r = (1-x-d)/(1+x-d)
% % = 1.4795
d = 0.5371;
RconstantMin = 0.6759;
RconstantMax = 1.4795;

ConstantDnear = 2;
ConstantDfar = 4;
dnear = gold_rate*ConstantDnear*ts;
