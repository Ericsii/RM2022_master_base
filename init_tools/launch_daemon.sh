#!/bin/sh

#### 设置工作目录
WORK_SPACE=/home/scurm/ros_workspace/scu_rm_ros
cd $WORK_SPACE

log_meg(){
        time=`date +"%F %T"`
        echo "[INFO] [$time]: $1"
}

process_monitor(){
  NUM=`ps -ef | grep $2 | grep -v grep | wc -l` 
  #Get the number of processes based on the process name

  if [ "${NUM}" -lt "1" ];then #If the number of processes is less than 1, start a new process
	log_meg "$1 $2 not exists, will launch..."
	# sh ${WORK_SPACE}/launch/launch_each.sh $1 $2
	gnome-terminal -- bash -c "bash ${WORK_SPACE}/launch/launch_each.sh $1 $2;exit;"
 #  If the number of processes is greater than 1, kill all processes and restart a new process with the same name
  elif [ "${NUM}" -gt "3" ];then
	log_meg "more than 1 $1,killall $1"
	kill -9 $(ps -ef|grep $1|grep -v grep|awk '{print $2}')
	# sh ${WORK_SPACE}/launch/launch_each.sh $1 $2
	gnome-terminal -- bash -c "bash ${WORK_SPACE}/launch/launch_each.sh $1 $2;exit;"
	# gnome-terminal -x bash -c "sh ~/daemon_script/run_total.sh $2;exec bash;"
  fi
#    Kill zombie processes
   NUM_STAT=`ps aux | grep $1 | grep T | grep -v grep | wc -l` 
 
  if [ "${NUM_STAT}" -gt "0" ];then
   	kill -9 $(ps -ef|grep $1 |grep -v grep|awk '{print $2}')
  fi
}

while true
do
    #### 设置启动节点
    process_monitor rm_infantry infantry_imu_up.launch.py
sleep 3s
    process_monitor rm_infantry infantry_all_up.launch.py
sleep 0.1
done
sleep 0.1

exit 0
