close all;
% clear all;
ConstantDefinition;
totalDataLenMilliSec = 91; % length of data in ms
numberOf6ms=15;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% datafilename='D:\peiqing\trackingLoop\LAdata\2004-8-19SSIIexternalClocking\SysExt_trig.txt';
% MAGSIGN = readTLA5202data(datafilename);
load TLA5202; % use load operation to save execution time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if numberOf6ms>floor(totalDataLenMilliSec/6)
    msgbox('data storage is not enough for required searching');
    return;
else
    [SVlist, all_ccn, all_frr, averaged_frr] = searchSV(numberOf6ms,MAGSIGN);
end
%load SavedsearchSV SVlist all_ccn all_frr averaged_frr; % use load operation to save execution time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


frrGeneratedbyTrackingLoop = [];
PresentSVnum = length(SVlist);
current_frr = averaged_frr;
allPhase1ms = [];
allUsedNoRarray=[];
for SVindex=1:PresentSVnum
    SVindex
    Phase1ms = [];
    debug_codeShiftNum = [];
    debug_codeShiftNumCurveFitting = [];
    temp = find(all_ccn(:,SVindex)>-inf);
    passed6ms = temp(1) - 1;
    VeryFirstSample = 5714*6*passed6ms + all_ccn(temp(1),SVindex);
    codePhaseAdjustment = 0;
    for TimeIndex = 1:HowManyLoopClosure
        TimeIndex
        fcSV = current_frr(SVindex);
        beginningSample =   codePhaseAdjustment + VeryFirstSample + (TimeIndex-1) * sampleNoIn1ms * trackingLoopPeriod;
        endingSample =      codePhaseAdjustment + VeryFirstSample + TimeIndex     * sampleNoIn1ms * trackingLoopPeriod - 1;
        if endingSample>length(MAGSIGN)
            break;
        end
        z10ms = MAGSIGN(beginningSample:endingSample)';
        
        [newfcSV, asdf, codeShiftNum, codeShiftNumCurveFitting, UsedNoRarray] = trackingLoop(SVlist(SVindex),fcSV,z10ms);
        allUsedNoRarray(SVindex, TimeIndex)=UsedNoRarray;
        debug_codeShiftNum = [debug_codeShiftNum codeShiftNum];
        debug_codeShiftNumCurveFitting = [debug_codeShiftNumCurveFitting codeShiftNumCurveFitting];
        codePhaseAdjustment = codePhaseAdjustment - codeShiftNum;
        Phase1ms = [Phase1ms asdf'];
        current_frr(SVindex) = newfcSV;
        frrGeneratedbyTrackingLoop(SVindex, TimeIndex) = newfcSV;
        if TimeIndex==5
            debug1 = 1;
        end
    end
    allPhase1ms(:,:,SVindex) = Phase1ms;
    if SVindex==1
        debug1 = 1;
    end
end
tt=1;
test1    