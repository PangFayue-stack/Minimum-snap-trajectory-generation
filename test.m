clear;
clc;
%12个点
path1 = [0 0 0; ...
         0.0 0.0 1.0 ; ...
         1.0 1.0 1.0 ; ...
         -1.0 2.0 1.0 ; ...
         1.0 3.0 1.0 ; ...
         -1.0 4.0 1.0 ; ...
         1.0 5.0 1.0 ; ...
         -1.0 6.0 1.0 ; ...
         1.0 7.0 1.0 ; ...
         -1.0 8.0 1.0 ; ...
         1.0 9.0 1.0 ; ...
         0.0 10.0 1.0 ; ];
load('C.mat')
% for i=1:11-1
%     find(C(8*i-3,:),1) == 2*i+4
%     find(C(8*i+1,:),1) == 2*i+4
% end
num = 11;
Ct = zeros(8 * num, 8 + 2 * (num - 1) + 3 * (num - 1));
for i=1:4
        Ct(i,i) = 1;
end
Ct(8*num-3:8*num,4+2*(num-1)+1:8+2*(num-1)) = eye(4);
for i=1:num-1
    Ct(8*i-3, 2*i+4) = 1;
    Ct(8*i+1, 2*i+4) = 1;
end
for i=1:num-1
   Ct(8*i-2:8*i,8+2*(num-1)+3*(i-1)+1:8+2*(num-1)+3*(i-1)+3) = eye(3);
   Ct(8*i+2:8*i+4,8+2*(num-1)+3*(i-1)+1:8+2*(num-1)+3*(i-1)+3) = eye(3);
end
info = Ct == C;
t = []
info = ones(size(info)) - info;
for i=1:8*num
    tmp = find(info(i,:),1);
    if tmp ~= 0
        t = [t tmp];
    else 
        t = [t,0];
    end
end