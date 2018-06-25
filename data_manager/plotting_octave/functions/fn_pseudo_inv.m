function pinv = fn_pseudo_inv(matrix, threshold)
    [U,S, V] = svd(matrix);
    len = length(S);
    Sinv = zeros(size(S));
    for i = 1:len
        if(S(i,i)>threshold)
            Sinv(i,i) = 1/S(i,i);
        end
    end
    pinv = U * Sinv * V';
end