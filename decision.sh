#! /bin/bash

# colcon build --symlink-install 
cmds=(  
	"ros2 launch rm_decision my_launch.py"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -ic "source ~/.bashrc;cd $(pwd);source install/setup.bash;$cmd;"
	sleep 0.2
done
