function ret = fn_add_path(path)
    path_name = sprintf('%s/functions', path);
    addpath(path_name);
%     path_name = sprintf('%s/../matlab_analysis_tools/readfile_script', path);
%     addpath(path_name);
%     path_name = sprintf('%s/../matlab_analysis_tools/Draw_script', path);    
%     addpath(path_name);
%     path_name = sprintf('%s/../matlab_analysis_tools/optimization_kinematics', path);    
%     addpath(path_name);
%     path_name = sprintf('%s/../matlab_analysis_tools/optimization_kinematics', path);
%     addpath(path_name);
end