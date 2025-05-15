while true; do
  echo "Waiting for release of STOP button"
  sleep 1
  python -m osgar.record config/matty-wait-for-start.json
  echo "Waiting for RORO25 termination"
  sleep 1
  python -m osgar.record --duration 10 config/matty-follow-road.json
done