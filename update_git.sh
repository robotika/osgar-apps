#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "Error: You must provide IP addres of robot"
  exit 1
fi

echo "Robot IP is: $1"

cd ~/git/osgar && git push robot@$1:/home/robot/git/bare/osgar.git
ssh robot@$1 "cd /home/robot/git/osgar && git pull"

cd ~/git/osgar-apps && git push robot@$1:/home/robot/git/bare/osgar-apps.git
ssh robot@$1 "cd /home/robot/git/osgar-apps && git pull"

