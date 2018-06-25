function w = fn_wgn(min, var)    
    rand1 = rand;
    rand2 = rand*pi*2;

    if(rand1 < 1e-100) 
        rand1 = 1e-100;
    end
    rand1 = -2*log(rand1);
    
    w = min + sqrt(var*rand1)*cos(rand2);
end