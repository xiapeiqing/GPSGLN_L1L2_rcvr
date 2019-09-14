close all
F=100;
aa = [];
t=0:1e-7:1/(8*F);
for thetat=0:0.1:2*pi
    top=sin(8*pi*F*t+thetat);
    top=sum(top)*1e-7;
    bottom = 1/(8*F);
    aa = [aa 10*log10(abs(top/bottom))];
end
plot(aa,'.')
disp(sprintf('calculation method from textbook, %f',aa(1)))

F=100;
bb = [];
t=0:1e-7:1/(8*F);
for thetat=0:0.1:2*pi
    top=1-sin(8*pi*F*t+thetat);
    top=sum(top)*1e-7;
    bottom = sin(8*pi*F*t+thetat);
    bottom=sum(bottom)*1e-7;
    
    bb = [bb 10*log10(abs(top/bottom))];
end
figure
plot(bb,'.')
disp(sprintf('my calculation method , %f',bb(1)))
