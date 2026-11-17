#!/bin/sh
# =====================================================================================
# deploy/install.sh  -  one-shot setup on the APB (Linux)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Copy the repo onto the board, then run:   sh deploy/install.sh
#   It creates the Python environment, installs the libraries, runs the quick tests
#   to make sure everything works on THIS computer, and sets the program to start at
#   boot (deploy/vision_hold.service). Safe to run again after updating the code.
#
# REFERENCES
#   - Python venv: https://docs.python.org/3/library/venv.html
#   - systemctl: https://www.freedesktop.org/software/systemd/man/systemctl.html
# =====================================================================================
set -e
cd "$(dirname "$0")/.."
DIR="$(pwd)"

python3 -m venv venv
venv/bin/pip install --upgrade pip
venv/bin/pip install -r requirements.txt

echo "--- Self-test on this board ---"
venv/bin/python -m tests.test_tracker
venv/bin/python -m tests.test_failsafe

echo "--- Installing the boot service ---"
sed "s|@USER@|$(id -un)|; s|@DIR@|$DIR|g" deploy/vision_hold.service | sudo tee /etc/systemd/system/vision_hold.service > /dev/null
sudo systemctl daemon-reload
sudo systemctl enable vision_hold
echo "Installed. Start now with: sudo systemctl start vision_hold"
echo "Benchmark this board with: venv/bin/python -m compute.benchmark"
