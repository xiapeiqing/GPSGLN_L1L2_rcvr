function code2 = digitizg(n,fs,offset,svnum)
% this function generates the local Code replica at a given carrier
% frequency for a given length.
% Input
%   n: length of signal to be generated, in sampling clock
%   fs: to be removed as it exists in ConstantDefinition
%   offset: offset in seconds to shift the replica
%   svnum: PRN number
% Output
%   code2: local Code replica sampled at HW clock
ConstantDefinition;
ts=1/fs;
tc=1/gold_rate;
 cmd1=codegen(svnum);
 code_in=cmd1;
 
 code_a=[code_in code_in code_in code_in];
 code_a=[code_a code_a code_a code_a];
 
 b=1:n;
 c=ceil((ts*b+offset)/tc);
 code=code_a(c);
 
 if offset>0 % at the program review stage, I guess this section is due to usage of ceil instead of floor previously.
     code2=[code(1) code(1:n-1)];
 else
     code2=[code(n) code(1:n-1)];
 end
