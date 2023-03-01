#!/bin/bash

rosws=catkin_ws
project_name=Sumitomo_L5G

sleep 15

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$project_name/autostart_scripts/auto_navigation.log

source /home/$USER/$project_name/autostart_scripts/ROS_CONFIG.txt
source /opt/ros/melodic/setup.bash
source /home/$USER/$rosws/devel/setup.bash

cd /home/$USER/$project_name/

while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting auto_navigation.py" >> $LOGFILE
		
		python -u auto_navigation.py >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
