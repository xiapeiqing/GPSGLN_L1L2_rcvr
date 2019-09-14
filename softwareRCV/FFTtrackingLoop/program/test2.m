
close all;
totalplot = 24;
for plotindex=0:totalplot-1
    curr_plotindex = mod(plotindex,8);
    if curr_plotindex==0
        figure;
    end
    subplot(4,2,curr_plotindex+1)
    shift = floor(totalplot/2);
    [newfcSV, asdf, codeShiftNum] = trackingLoop(SVlist(SVindex),fcSV,MAGSIGN(beginningSample+plotindex-shift:endingSample+plotindex-shift)')
    plot(asdf,'.')
    title(sprintf('mandatory %d, codeShiftNum %d',plotindex-shift,codeShiftNum))
end






















