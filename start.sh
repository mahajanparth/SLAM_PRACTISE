#!/usr/bin/env bash
# Launch the SLAM Practice judge and open it in the browser.
cd "$(dirname "$0")"
PORT="${1:-8008}"
if curl -s "http://127.0.0.1:$PORT/api/catalog" >/dev/null 2>&1; then
  echo "Already running at http://127.0.0.1:$PORT"
else
  python3 app/server.py --port "$PORT" &
  sleep 1
fi
(xdg-open "http://127.0.0.1:$PORT" >/dev/null 2>&1 &)
wait
