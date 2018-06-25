function count = fn_num_large_singular_value(matrix, threshold)
    s = svd(matrix);
    
    count = 0;
    len = length(s);
    for i = 1:len
        if(s(i)>threshold)
            count = count +1;
        end
    end
end