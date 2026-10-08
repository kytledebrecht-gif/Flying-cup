#!/usr/bin/env bash
set -euo pipefail
if docker ps -a --format '{{.Names}}' | grep -qx potato-navigation; then
  docker stop -t 8 potato-navigation >/dev/null
  echo "Navigation stopped; zero command will be selected by the existing mux/watchdog."
else
  echo "potato-navigation is not running."
fi
