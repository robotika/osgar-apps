while true; do
  echo
  echo "Waiting for release of STOP button"
  sleep 1
  python -m osgar.record config/matty-wait-for-start.json

  echo
  echo "Waiting for RORO25 termination"
  sleep 1
  python -m osgar.record config/matty-follow-road.json --param app.dist_limit=3.20

  echo "Turn 180deg"
  sleep 1
  python -m osgar.record ~/git/osgar/config/matty-go.json --param app.steering_deg=45 app.timeout=6.5

  echo
  echo "Waiting for RORO25-back termination"
  sleep 1
  python -m osgar.record config/matty-follow-road.json

done