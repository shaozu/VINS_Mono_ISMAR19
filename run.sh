#!/bin/bash
trap 'kill $VINS_PID; kill $BAG_PID; exit' INT

source ~/.bashrc
# change the path to your folder that contains bag file in it.
dataset_path=/home/shaozu/data/ismar_ar/second_round
result_pose_path=/home/shaozu/output/vins_result_odometry.txt
result_time_path=/home/shaozu/output/vins_result_time.txt
result_save_path=/home/shaozu/data/ismar_ar/second_round/result
files=$(ls $dataset_path)
round=5

pkill -f vins_estimator && sleep 2;
for filename in $files; do
    if [[ $filename == *.bag ]]; then
        sequence="${filename%.*}"
        seq_dir="${result_save_path}/${sequence}"
        mkdir -p ${seq_dir}
        for i in `seq 0 $((round-1))`; do 
            echo "Pass ${i}: runing" $filename "..."
            success=1
            roslaunch vins_estimator ismar_xiaomi.launch &
            VINS_PID=$!
            sleep 3
            rosbag play -q $dataset_path/$filename &
            BAG_PID=$!
            while [ $(ps -ef |grep rosbag |grep -v "grep" |wc -l) -ne 0 ]; do
                if [ $(ps -ef |grep vins_estimator |grep -v "grep" |wc -l) -ne 2 ]; then  # vins vio failure
                    success=0
                    break
                fi 
                sleep 1;
            done
            echo $filename "finish ..."
            echo "shut down vins_estimator ..."
            pkill -f vins_estimator
            pkill -f rosbag
            if [ $success == 1 ]; then
                echo "vins_estimator shut down! Save results..."
                mv ${result_pose_path} ${seq_dir}/${i}-pose.txt
                mv ${result_time_path} ${seq_dir}/${i}-time.txt
            fi
        done
    fi
done
