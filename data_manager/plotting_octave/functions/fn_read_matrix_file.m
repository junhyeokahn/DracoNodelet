function ret = fn_read_matrix_file(path, name, num_column)
    fname = sprintf('%s/%s.txt', path, name);
    fileID = fopen(fname, 'r');
    
    ret = cell(1);
    
    k = 1;
    
    while(~feof(fileID))
        switch num_column
            case 1
                matrix = fscanf(fileID, '%f', [1 inf]);
            case 3
                matrix = fscanf(fileID, '%f %f %f', [3 inf]);
            case 4
                matrix = fscanf(fileID, '%f %f %f %f', [4 inf]);
            case 5
                matrix = fscanf(fileID, '%f %f %f %f %f', [5 inf]);
            case 6
                matrix = fscanf(fileID, '%f %f %f %f %f %f', [6 inf]);  
            case 12
                matrix = fscanf(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f', [12 inf]);
        end
        ret{k} = matrix;
        k = k+1;
    end
    fclose(fileID);
end