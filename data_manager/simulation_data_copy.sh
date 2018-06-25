#! /bin/bash
PATH_PACKAGE="/home/apptronik/ros/apptronik_two_dt/data_manager"

# Move experiment data to the other folder to plot data
cp $PATH_PACKAGE/experiment_data/* $PATH_PACKAGE/experiment_data_check/

# Remove every data for the next experiment
rm $PATH_PACKAGE/experiment_data/*

exit 0
