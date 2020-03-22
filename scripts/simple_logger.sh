#!/bin/bash
#Usage: log_message "Some Text"
#Usage: echo "a command" | log_message
boot_file="/var/log/output/"$(hostname)"_log.txt"
mkdir -p /var/log/output/
touch $boot_file
sudo chown robot:robot $boot_file
get_TimeStamp () { Time="["$(date +%d-%b-%Y)" "$(date +%H:%M:%S)"/"`awk '{print $1}' /proc/uptime`"]" ; }
function log_message {
  get_TimeStamp
  if [ -n "$1" ]; then
      MESSAGE="$1"
      echo -e "${Time} $MESSAGE" | tee -a $boot_logfile
  else
      MESSAGE=$(tee)
      echo -e "${Time} exec:$MESSAGE" >> $boot_file
      eval $MESSAGE >> $boot_file 2>&1 &
  fi
}


