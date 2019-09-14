function [retValue,retIndex]=PickUpSimilar(data,threshold)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% M input variables, if the difference is less than input parameter
% "threshold", we treat them as the same. return the largest group of similar variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
datanumber = length(data);
index=zeros(datanumber,datanumber);
for kk=1:datanumber
    for jj=1:datanumber
        if abs(data(kk)-data(jj))<threshold
            index(kk,jj)=1;
        end
    end
end
similarCounter=sum(index);
[Y,Ix] = max(similarCounter);
retIndex = find(index(Ix,:)==1);
retValue = data(retIndex);









