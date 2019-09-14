function newArray = cyclicShift(array,L1R2,shiftNumber)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% input parameter "array " is shifted to the left or right by "shiftNumber"
% chips. The direction is decided by "L1R2". "1" indicates shifting to the
% left, "2" indicates shifting to the right.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
arraylength = length(array);
if shiftNumber>=arraylength
    msgbox('error in executing function cyclicShift, shift number is too large');
end
newArray = [];
switch L1R2
    case 1
        newArray = array(shiftNumber+1:arraylength);
        newArray = [newArray array(1:shiftNumber)];
    case 2
        newArray = array(1:arraylength-shiftNumber);
        newArray = [array(arraylength-shiftNumber+1:arraylength) newArray];
    otherwise
        msgbox('error in executing function cyclicShift, 2nd input parameter is meaningless');
end










