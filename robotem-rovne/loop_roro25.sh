while true; do
  echo
  echo "Waiting for release of STOP button"
  sleep 1
  python -m osgar.record config/matty-wait-for-start.json

  echo
  echo "Waiting for RORO25-back termination"
  sleep 1
  python -m osgar.record config/matty-follow-road.json

done
