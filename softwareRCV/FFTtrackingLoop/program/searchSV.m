function [SVlist, all_ccn, all_frr, averaged_frr] = searchSV(numberOf6ms,MAGSIGN)
% this function uses the real mode data stored in MAGSIGN to detect signal
% energy presence.

ConstantDefinition;
% TrueavailableSV=[15 18 22 26 29]
firstSample = 1; % the starting sampling clock index to run signal search. It uses matlab convention of "1" being the first.
wrongXoccurTimes=[]; % length 32 vector, element No.x correponds to GPS PRN x.
Xratio=[];
all_ccn=[];
all_frr=[];
for k=1:numberOf6ms
    startingSample = 6*sampleNoIn1ms*(k-1)+firstSample;
    [wrongXoccurTimesTemp,XratioTemp,ccn,frr] = srch5ms(fs,fc,startingSample,MAGSIGN);
    wrongXoccurTimes=[wrongXoccurTimes;wrongXoccurTimesTemp]; % 15*32 matrix, 15 is the number of available 6ms data, 32 is SV number
    Xratio=[Xratio;XratioTemp]; % 15*32 matrix, 15 is the number of available 6ms data, 32 is SV number
    all_ccn=[all_ccn;ccn]; % 15*32 matrix, 15 is the number of available 6ms data, 32 is SV number
    all_frr=[all_frr;frr]; % 15*32 matrix, 15 is the number of available 6ms data, 32 is SV number
end
Xratio= sum(Xratio); % sum over the 15 available 6ms segments, to boost up the detectability
SVlist=find(Xratio>mean(Xratio)); % above the average means satellite presence
PresentSVnum = length(SVlist); % indicate how many satellite is present
averaged_frr = [];
temp_frr = all_frr;
temp_all_ccn = all_ccn;

all_frr = [];
all_ccn = [];
for kk = 1 : numberOf6ms
    for jj = 1 : PresentSVnum
        all_frr(kk,jj)=NaN;
        all_ccn(kk,jj)=NaN;
    end
end
for kk=1:PresentSVnum
    [retValue,retIndex]=PickUpSimilar(temp_frr(:,SVlist(kk)),3*std(temp_frr(:,SVlist(kk))));
    averaged_frr = [averaged_frr mean(retValue)];
    all_frr(retIndex,kk)=retValue;
    all_ccn(retIndex,kk)=temp_all_ccn(retIndex,SVlist(kk));
end
save SavedsearchSV SVlist all_ccn all_frr averaged_frr;
