#!/bin/sh
# =====================================================================================
# deploy/install.sh  -  one-shot setup on the APB (Linux)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Copy the repo onto the board (rsync -avP DroneCode root@192.168.75.1:~ over the USB-C
#   cable, APB manual p.10), then on the board:   sh deploy/install.sh
#   It creates the Python environment, installs the libraries, runs the quick tests
#   to make sure everything works on THIS computer, and INSTALLS (but does not enable) the
#   boot service deploy/vision_hold.service. Enable it only for event day. Safe to rerun.
#   No internet on the board? See docs/hardware.md "Offline install".
#
# REFERENCES
#   - Python venv: https://docs.python.org/3/library/venv.html
#   - systemctl: https://www.freedesktop.org/software/systemd/man/systemctl.html
# =====================================================================================
set -e
cd "$(dirname "$0")/.."
DIR="$(pwd)"
SUDO=""
[ "$(id -u)" -ne 0 ] && SUDO="sudo"          # the APB logs in as root: no sudo needed there

python3 -m venv venv
venv/bin/pip install --upgrade pip
venv/bin/pip install -r requirements.txt

echo "--- Self-test on this board ---"
venv/bin/python -m tests.test_tracker
venv/bin/python -m tests.test_handover

echo "--- Installing (NOT enabling) the boot service ---"
sed "s|@USER@|$(id -un)|; s|@DIR@|$DIR|g" deploy/vision_hold.service | $SUDO tee /etc/systemd/system/vision_hold.service > /dev/null
$SUDO systemctl daemon-reload
echo "Installed. For event day only:  $SUDO systemctl enable --now vision_hold"
echo "Before ANY manual run:           $SUDO systemctl stop vision_hold"
echo "Benchmark this board:            venv/bin/python -m compute.benchmark"
