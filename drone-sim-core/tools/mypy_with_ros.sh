#!/bin/bash
set -e
export PYTHONPATH=$(find install -type d -name site-packages | paste -sd: -):$PYTHONPATH
mypy src tests --exclude 'setup.py' "$@" 