function [all_G1, all_G2, all_CAcode, risingEdge1KHz] = CAcodeGeneratorAnyTime( SVnumber1to32, mode )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
all_G1=[];
all_G2=[];
all_G1_outbit=[];
all_CAcode=[];
risingEdge1KHz=[];

% initialization
timeaxis=(1:sequenceLength)/1.023e6;
G1=ones(1,10);
G2=ones(1,10);

for index=1:sequenceLength
    G1outBit = G1(10);
    newG1bit1 = xor(G1(3),G1(10));
    G1(2:10) = G1(1:9);
    
    G1(1) = newG1bit1;
    if mode == 1
        all_G1 = [all_G1;G1];
        all_G1_outbit = [all_G1_outbit; G1outBit];
    end
    
    if mode == 1
        all_G2 = [all_G2;G2];
        if (sum(G2)==10)
            risingEdge1KHz = [risingEdge1KHz; 1];
        else
            risingEdge1KHz = [risingEdge1KHz; 0];
        end
    end
    G2outBit = xor(G2(2),G2(6));
    newG2bit1 = xor(G2(2),G2(3));
    newG2bit1 = xor(newG2bit1,G2(6));
    newG2bit1 = xor(newG2bit1,G2(8));
    newG2bit1 = xor(newG2bit1,G2(9));
    newG2bit1 = xor(newG2bit1,G2(10));
    G2(2:10) = G2(1:9);
    G2(1) = newG2bit1;
    
    thisCAcode = xor(G1outBit,G2outBit);
    all_CAcode = [all_CAcode; thisCAcode];
end



