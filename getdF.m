function d_F = getdF(path,i)
    t = path(:,i);
    n = size(t,1)-1;
    d_F = zeros(2 * n + 6,1);
    d_F(1:4) = [t(1,1);0;0;0];
    for i=1:size(t,1)-2
        d_F(4+i*2-1,1)=t(i+1);
        d_F(4+i*2,1)=t(i+1);
    end
    d_F(4+2*(n-1)+1:8+2*(n-1),1) = [t(end);0;0;0];
end