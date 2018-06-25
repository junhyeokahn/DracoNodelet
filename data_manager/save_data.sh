#! /bin/bash
folder_name=$(date +%Y%m%d_%H_%M_%S)
target_folder="/home/apptronik/MyCloud/Apptronik/TMech_Rebuttal/ContinuousCurrent"
data_location="/home/apptronik/ros/apptronik_two_dt/data_manager"
config_location="/home/apptronik/ros/apptronik_two_dt/main/config"
mkdir -p ${target_folder}/${folder_name}

rm -rf ${target_folder}/../LatestData/*
cp ${data_location}/experiment_data/* ${target_folder}/../LatestData/
# Move experiment data to the other folder to plot data
cp ${data_location}/experiment_data/* ${data_location}/experiment_data_check/

# Save Data, Scenario, video
cp ${data_location}/experiment_data/* ${target_folder}/${folder_name}/

# Remove every data for the next experiment
rm ${data_location}/experiment_data/*

exit 0
