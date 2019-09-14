figure;
for kk = 1:21
    plot(abs(yy(kk,:)));
    title(sprintf('freq=%d',inst_fr(kk)))
    pause;
end