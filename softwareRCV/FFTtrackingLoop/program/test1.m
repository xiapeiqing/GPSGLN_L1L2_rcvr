close all
% figure
% hold on
% plot(mod(zz1,2),'*-')
% % plot(ones(1,length(zz1))*1.5)
% % plot(ones(1,length(zz1))*0.5)
% % plot(ones(1,length(zz1))*1)
% plot(tempdiscontinuity,'y.-')
% plot(temp,'r.-')
% plot(unwrap(discontinuity1ms),'m.-')
% legend('temp-dis','tempdiscontinuity','temp','unwrap(discontinuity1ms)');
% hold off
figureNum = size(allPhase1ms,3);
subFigNum = size(allPhase1ms,2);
rowNum = ceil(subFigNum/2);
for index = 1:figureNum
    debug = allPhase1ms(:,:,index);
    figure;
    for subindex = 1:subFigNum
        subplot(rowNum,2,subindex)
        plot(debug(:,subindex),'.')
        ylabel(sprintf('PRN %d, %d to %dms',SVlist(index), (subindex-1)*10, subindex*10));
    end
end


