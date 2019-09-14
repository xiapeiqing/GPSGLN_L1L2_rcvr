function ca_used = codegen(svnum)
gs2=[5;6;7;8;17;18;139;140;141;251;252;254;255;256;257;258;469;470;471;472;473;474;509;512;513;514;515;516;859;860;861;862];
g2shift=gs2(svnum,1);

reg=-1*ones(1,10);
for i=1:1023
    g1(i)=reg(10);
    save1=reg(3)*reg(10);
    reg(1,2:10)=reg(1:1:9);
    reg(1)=save1;
end

reg=-1*ones(1,10);
for i=1:1023
    g2(i)=reg(10);
    save2=reg(2)*reg(3)*reg(6)*reg(8)*reg(9)*reg(10);
    reg(1,2:10)=reg(1:1:9);
    reg(1)=save2;
end

g2tmp(1,1:g2shift)=g2(1,1023-g2shift+1:1023);
g2tmp(1,g2shift+1:1023)=g2(1,1:1023-g2shift);
g2=g2tmp;

ss_ca=g1.*g2;
ca_used=-ss_ca;
