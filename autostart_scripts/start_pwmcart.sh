#!/bin/bash

rosws=catkin_ws
project_name=Sumitomo_L5G

sleep 10

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$project_name/autostart_scripts/pwmcart.log

source /home/$USER/$project_name/autostart_scripts/ROS_CONFIG.txt
source /opt/ros/melodic/setup.bash
source /home/$USER/$rosws/devel/setup.bash

while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting jmoab-ros pwmcart.py" >> $LOGFILE
		
		rosrun jmoab-ros pwmcart.py >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
