function TorF = prime(a)
TorF =1;
for kk=2:a-1
    if (mod(a,kk)==0)
        TorF = 0;
        break;
    end
end
if (TorF == 1)
    disp('input parameter is prime')
else
    disp('input parameter is NOT prime')
end