#!/bin/bash
set -e

# Web visualization server startup
echo "[entrypoint-web-viz] Starting web visualization server..."

# Change to web_viz directory
cd /workspace/web_viz

# Start the web server
python3 -m flask run --host=0.0.0.0 --port=8080 || {
  echo "Web visualization server failed to start"
  exit 1
} 