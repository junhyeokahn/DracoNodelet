#! /bin/bash
folder_name=$(date +%Y%m%d_%H_%M_%S)
target_folder="/home/apptronik/AnkleKneeExperiment"
data_location="/home/apptronik/ros/DracoNodelet/AnkleKneeController/"

mkdir -p ${target_folder}/${folder_name}

cp ${data_location}/ExperimentData/* ${data_location}/ExperimentDataCheck/
cp ${data_location}/ExperimentData/* ${target_folder}/${folder_name}/
rm ${data_location}/ExperimentData/*

#exit 0
