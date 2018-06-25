function ret = fn_read_matrix_file_slow(path, name, num_column)
    fname = sprintf('%s/%s.txt', path, name);
    fileID = fopen(fname, 'r');
    
    ret = cell(1);
    
    i = 1;
    k = 1;
    
    matrix = zeros(1);
    while(~feof(fileID))
        first_term = fscanf(fileID, '%s', 1);
        if( isequal(first_term, 'end'))  
            ret{k} = matrix;    
            i = 1;
            k = k+1;
            matrix = zeros(1);
            matrix(i,1) = str2num(fscanf(fileID, '%s', 1));
        elseif(size(str2num(first_term)) <1)
                break;
        else
            matrix(i,1) = str2num(first_term);
        end
    
        for j = 2:num_column
            tmp = str2num(fscanf(fileID, '%s', 1));

            if(length(tmp) <1)
                break;
            end
            matrix(i,j) = (tmp);
        end
        i = i+1;
    end
    fclose(fileID);
end