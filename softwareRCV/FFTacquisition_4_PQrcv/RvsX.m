close all
x=-1:0.01:1;
d=0.5371;
r = (1-x-d)./(1+x-d);
figure
[ymax,maxin]=max(r);
[ymin,minin]=min(r);
hold on
plot(x,r,'.');
text(x(maxin)+0.03,ymax+10,sprintf('x= %f',x(maxin)))
text(x(minin)+0.03,ymin+10,sprintf('x= %f',x(minin)))
hold off

r=0:0.1:10000;
d=0.5371;
x = (1-r)*(1-d)./(1+r);
figure
% [ymax,maxin]=max(r);
% [ymin,minin]=min(r);
% hold on
semilogx(r,x,'.');
xlabel('r')
ylabel('x')
% text(x(maxin)+0.03,ymax+10,sprintf('x= %f',x(maxin)))
% text(x(minin)+0.03,ymin+10,sprintf('x= %f',x(minin)))
% hold off
