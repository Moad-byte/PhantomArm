#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
sudo apt update
sudo apt install -y python3-venv python3-pip libgl1 libglfw3
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m pytest -v tests
.venv/bin/python -m phantomarm.verify_model
