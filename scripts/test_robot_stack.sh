#!/usr/bin/env bash
set -euo pipefail

HOST=${1:-fordward.local}

echo "== Checking rosbridge (9090) on $HOST =="
if command -v nc >/dev/null 2>&1; then
  if nc -z -w2 "$HOST" 9090; then echo "rosbridge: OK"; else echo "rosbridge: UNREACHABLE"; fi
else
  echo "Skipping netcat check (nc not installed)."
fi

echo "== Checking web_video_server (8080) on $HOST =="
if curl -s -I --connect-timeout 2 "http://$HOST:8080/stream_viewer" | head -n1 | grep -qE "^HTTP/"; then
  echo "web_video_server: OK"
else
  echo "web_video_server: UNREACHABLE"
fi

echo "== Checking rosboard (8888) on $HOST =="
if curl -s -I --connect-timeout 2 "http://$HOST:8888" | head -n1 | grep -qE "^HTTP/"; then
  echo "rosboard: OK"
else
  echo "rosboard: UNREACHABLE"
fi

echo "Done."

