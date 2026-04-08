if [ "$#" -eq 2 ]; then
  if [ "$2" -eq 1 ]; then
    echo "command = GOAL"
  elif [ "$2" -eq 2 ]; then
    echo "command = EXPLORE"
  elif [ "$2" -eq 3 ]; then
    echo "command = PATROL"
  elif [ "$2" -eq 4 ]; then
    echo "command = GO HOME"
  fi
  rostopic pub -1 /instruction quadrotor_msgs/Instruction "$1" "$2"
else
  echo "command param num error， num = $#"
fi