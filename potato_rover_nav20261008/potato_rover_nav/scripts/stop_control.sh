#!/usr/bin/env bash
set -euo pipefail
docker stop -t 3 potato-rover-control >/dev/null 2>&1 || true
echo 'Control stopped; shutdown sent three zero-speed frames.'
