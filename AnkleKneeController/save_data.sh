#! /bin/bash
folder_name=$(date +%Y%m%d_%H_%M_%S)
target_folder="/home/apptronik/AnkleKneeExperiment"
data_location="/home/apptronik/ros/DracoNodelet/AnkleKneeController/"

mkdir -p ${target_folder}/${folder_name}

cp ${data_location}/ExperimentData/* ${data_location}/ExperimentDataCheck/
cp ${data_location}/ExperimentData/* ${target_folder}/${folder_name}/
rm ${data_location}/ExperimentData/*

#cd ExperimentDataCheck
#for data in debug_bus_voltage.txt debug_knee_effort.txt debug_knee_qdot.txt debug_knee_q.txt
