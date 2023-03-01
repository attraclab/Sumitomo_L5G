#!/bin/bash

rosws=catkin_ws
project_name=Sumitomo_L5G

sleep 30

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$project_name/autostart_scripts/local_data_server.log

cd /home/$USER/web_dev/sumitomo_l5g/python/

while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting python3 local_data_server.py" >> $LOGFILE
		
		python3 -u local_data_server.py >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
