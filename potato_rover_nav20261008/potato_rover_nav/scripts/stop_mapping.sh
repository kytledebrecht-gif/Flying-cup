#!/usr/bin/env bash
set -euo pipefail
if docker ps -a --format '{{.Names}}' | grep -qx potato-mapping; then
  docker stop -t 8 potato-mapping >/dev/null
  echo "Mapping stopped; MID-360 stream closed normally."
else
  echo "potato-mapping is not running."
fi
