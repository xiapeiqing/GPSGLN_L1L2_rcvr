load TLA5202;
rawLength = length(MAGSIGN);
IQlength = floor(rawLength/2);

temp = (1:IQlength)*2;
QdataStream = MAGSIGN(temp);

temp = temp -1;
IdataStream = MAGSIGN(temp);

clear MAGSIGN;

halfIQlength = floor(IQlength/2);
temp = (1:halfIQlength)*2;
QdataStream(temp) = -QdataStream(temp);
temp = temp -1;
IdataStream(temp) = -IdataStream(temp);

