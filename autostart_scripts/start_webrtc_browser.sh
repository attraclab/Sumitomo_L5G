#!/bin/bash

rosws=catkin_ws
project_name=Sumitomo_L5G

sleep 35

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$project_name/autostart_scripts/browser.log

while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting chromium-browser" >> $LOGFILE
		
		chromium-browser  http://localhost/sumitomo_l5g/offer.html >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
