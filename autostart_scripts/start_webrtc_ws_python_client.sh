#!/bin/bash

rosws=catkin_ws
project_name=Sumitomo_L5G

sleep 30

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$project_name/autostart_scripts/ws_python_client.log

cd /home/$USER/web_dev/sumitomo_l5g/python/

while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting python3 ws_python_client.py" >> $LOGFILE
		
		python3 -u ws_python_client.py >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
