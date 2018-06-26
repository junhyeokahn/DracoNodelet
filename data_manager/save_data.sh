#! /bin/bash
folder_name=$(date +%Y%m%d_%H_%M_%S)
target_folder="/home/apptronik/AnkleKneeExperiment"
data_location="/home/apptronik/ros/DracoNodelet/data_manager"
mkdir -p ${target_folder}/${folder_name}

# Move experiment data to the other folder to plot data
cp ${data_location}/experiment_data/* ${data_location}/experiment_data_check/

# Save Data, Scenario, video
cp ${data_location}/experiment_data/* ${target_folder}/${folder_name}/

# Remove every data for the next experiment
rm ${data_location}/experiment_data/*

exit 0
