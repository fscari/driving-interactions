#!/bin/bash

# check args

if [[ $# -eq 2 ]]; # Check if the number of arguments is 2
then
  exp_nr="$1"
  session_nr="$2"
else
  echo "Wrong arg count"
  exit 1 # Exit the script if the argument count is wrong
fi

#python3 exp_setup_vehicle1.py "$exp_nr" "$session_nr" &
#python3 exp_setup_vehicle2.py "$exp_nr" "$session_nr"

# Run scripts in new terminal windows
gnome-terminal -- python3 exp_setup_vehicle1.py "$exp_nr" "$session_nr"
sleep 1
gnome-terminal -- python3 exp_setup_vehicle2.py "$exp_nr" "$session_nr"