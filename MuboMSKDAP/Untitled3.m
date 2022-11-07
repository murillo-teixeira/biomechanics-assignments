t = 15.43;
str0 = '...Time...|';
str1 = '............................................';
str2 = '.Body (';
str3 = ').';
str4 = '.......X........|.......Y........|';
str5 = '.......Xd.......|.......Yd.......|';
str6 = '......Xdd.......|......Ydd.......|';
strA = str0;
strB = '          |';
strC = sprintf('%10.4f',t);
for i=1:6
    str  = sprintf('%03d',i);
    strA = strcat(strA,'.',str1,str2,str,str3,str1,'|');
    strB = strcat(strB,str4,str5,str6);
    str  = sprintf('%16.9d',t);
    strC = char(strcat(strC,{' '},str));
end

strA
strB
strC
%       12345678901234567890123456789012345678901234
Theta = char(hex2dec('03B8')); 
Omega = fprintf([Theta 'd']);