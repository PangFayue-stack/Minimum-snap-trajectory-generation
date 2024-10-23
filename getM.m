function M = getM(n_seg, n_order, ts)
    M = [];
    d_order = 4;
    for j = 1:n_seg
        M_k = [];
        for k=0:d_order-1
            M_k(k+1,k+1) = factorial(k);
            for i=k:n_order
                M_k(k+1+4,i+1)=factorial(i)/factorial(i-k)*ts(j)^(i-k);
            end
        end
        M = blkdiag(M, M_k);
    end
end