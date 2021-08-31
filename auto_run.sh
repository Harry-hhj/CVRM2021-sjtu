#!/bin/bash
echo nvidia | sudo -S nvpmodel -m 2
echo nvidia | sudo -S jetson_clocks --fan
cd /home/nvidia/Workspace/CVRM2021-Sentry-Down/script
while true
do
	echo nvidia | sudo -S ../build/CVRM2021 app.py -s robot_io.py sensors_io.py autoaim.py | tee "../data/$(date).txt" 2>&1
done

