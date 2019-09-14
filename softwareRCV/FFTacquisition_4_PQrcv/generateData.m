function data = generateData(doppler,fs,fc,CNo,datalengthms,SVnum)
timeaxis = [];
for time=0:1/fs:datalengthms*1e-3-1/fs
    timeaxis = [timeaxis time];
end
CWsignal = sin(2*pi*(fc+doppler)*timeaxis);
code=digitizg(datalengthms*1e-3*fs,fcSV,0,SVnum);
