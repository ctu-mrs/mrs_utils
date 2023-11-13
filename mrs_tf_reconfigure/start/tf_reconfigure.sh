#!/bin/bash

SESSION_NAME=tf_reconfigure

# following commands will be executed first, in each window
pre_input="export UAV_NAME=$UAV_NAME; export ATHAME_ENABLED=0"

# define commands
# 'name' 'command'
input=(
  'Roscore' 'roscore
'
  'TfReconfigure' "waitForRos; roslaunch tf_reconfigure tf_reconfigure.launch
"
  'RVIZ' "waitForRos; roscd tf_reconfigure; rosrun rviz rviz -d rviz/tf_reconfigure.rviz
"
  'Reconfigure' "waitForRos; rosrun rqt_reconfigure rqt_reconfigure
"
  'Layout' "waitForRos; sleep 5; ~/.i3/layout_manager.sh layout.json
"
)

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  SESSION_NAME="$(tmux display-message -p '#S')"
fi

# create arrays of names and commands
for ((i=0; i < ${#input[*]}; i++));
do
  ((i%2==0)) && names[$i/2]="${input[$i]}" 
	((i%2==1)) && cmds[$i/2]="${input[$i]}"
done

# run tmux windows
for ((i=0; i < ${#names[*]}; i++));
do
	tmux new-window -t $SESSION_NAME:$(($i+10)) -n "${names[$i]}"
done

sleep 2

# send commands
for ((i=0; i < ${#cmds[*]}; i++));
do
	tmux send-keys -t $SESSION_NAME:$(($i+10)) "${pre_input};
${cmds[$i]}"
done

sleep 2

tmux select-window -t $SESSION_NAME:11
tmux -2 attach-session -t $SESSION_NAME

clear
