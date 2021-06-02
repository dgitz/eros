#!/bin/bash
#Usage: 
source ./scripts/simple_logger.sh
launch_list_file="~/storage/stage/device_launch_list"
while IFS= read -r line
do
  items=$(echo $line | tr ";" "\n")
  counter=0
  workspace_path=""
  workspace_launch_package=""
  launch_file=""
  for item in $items
  do
    # Look for comments
    if [ "${item:0:1}" = "#" ]; then
      break
    fi
    if [ $counter -eq 0 ];then
      workspace_path=$item
    elif [ $counter -eq 1 ];then
      workspace_launch_package=$item
    elif [ $counter -eq 2 ];then
      launch_file=$item
    fi
    ((counter=counter+1))
  done
  if [ $counter -eq 3 ];then
      cd "${workspace_path}"
      source devel/setup.bash
      echo "Launching WS: "${workspace_path}" Package:  "${workspace_launch_package}" Launch File: "${launch_file}
      launch_command="roslaunch "${workspace_launch_package}" "${launch_file}" > /dev/null 2>&1 &"
      eval $launch_command
  fi
done < "$launch_list_file"
