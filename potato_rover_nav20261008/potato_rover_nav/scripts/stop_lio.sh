#!/usr/bin/env bash
set -euo pipefail
docker stop -t 5 potato-lio >/dev/null 2>&1 || true
echo 'LIO stopped.'
