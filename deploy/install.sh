#!/bin/sh
# =====================================================================================
# deploy/install.sh  -  one-shot setup on the APB (Linux)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Copy the repo into /data on the board over the USB-C cable:
#       rsync -avP DroneCode root@192.168.75.1:/data/
#   then on the board:   cd /data/DroneCode && sh deploy/install.sh
#   WHY /data (Stage 2 rule): the root filesystem is almost full and /tmp is lost at reboot;
#   /data also survives software updates (APB manual p.18).
#   The board has no internet and no compiler: put ready-made library files (aarch64, Python
#   3.12 "wheels") in a folder called wheels/ next to this repo's requirements.txt; the script
#   then installs only from that folder. Make it on a laptop WITH internet:
#       pip download -r requirements.txt -d wheels --only-binary=:all: #           --platform manylinux2014_aarch64 --python-version 3.12
#   It creates the Python environment, installs the libraries, runs the quick tests
#   to make sure everything works on THIS computer, and INSTALLS (but does not enable) the
#   boot service deploy/vision_hold.service. Enable it only for event day. Safe to rerun.
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

case "$DIR" in
    /data/*) ;;
    *) [ -d /data ] && echo "WARNING: this repo is in $DIR. Stage 2: keep it in /data (root filesystem is almost full)." ;;
esac

python3 -m venv --system-site-packages venv      # reuse the numpy that ships with the board
if [ -d wheels ]; then
    venv/bin/pip install --no-index --find-links wheels -r requirements.txt
else
    venv/bin/pip install -r requirements.txt     # laptop / board with internet
fi

echo "--- Self-test on this board ---"
venv/bin/python -m tests.test_tracker
venv/bin/python -m tests.test_handover
venv/bin/python -m tests.test_betaflight

echo "--- Installing (NOT enabling) the boot services ---"
for unit in vision_hold msp_sender; do
    sed "s|@USER@|$(id -un)|; s|@DIR@|$DIR|g" deploy/$unit.service | $SUDO tee /etc/systemd/system/$unit.service > /dev/null
done
$SUDO systemctl daemon-reload
echo "Installed. Betaflight (stock):   $SUDO systemctl enable --now msp_sender vision_hold"
echo "ArduCopter (if flashed):         edit ExecStart in deploy/vision_hold.service, rerun this, enable vision_hold only"
echo "Before ANY manual run:           $SUDO systemctl stop vision_hold msp_sender"
echo "Watch the live estimate:         journalctl -u vision_hold -f"
echo "Benchmark this board:            venv/bin/python -m compute.benchmark"
