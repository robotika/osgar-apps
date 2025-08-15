#!/bin/bash

if [ "$#" -lt 2 ]; then
  echo "Error: <wait> <dtc> [optional DTC params]"
  exit 1
fi

# If script reaches here, exactly one parameter was provided
echo "Parameter is: $1, $2"

wait_cfg="$1"
dtc_cfg="$2"

# Shift the first two parameters so $@ now contains only optional parameters for the second command
shift 2

while true; do
  echo
  echo "Waiting for release of STOP button"
  sleep 1
  python -m osgar.record $wait_cfg

  echo
  echo "Waiting for DTC run termination"
  sleep 1
  python -m osgar.record $dtc_dtc_cfg "$@"

done
