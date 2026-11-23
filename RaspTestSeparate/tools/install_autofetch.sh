#!/bin/bash
# Install (or reinstall) the pico-autofetch systemd user service on this laptop. No sudo needed.
#   bash tools/install_autofetch.sh            install and start
#   bash tools/install_autofetch.sh remove     stop and remove
set -e
UNIT=~/.config/systemd/user/pico-autofetch.service
if [ "$1" = "remove" ]; then
    systemctl --user disable --now pico-autofetch.service 2>/dev/null || true
    rm -f "$UNIT"; systemctl --user daemon-reload; echo "removed"; exit 0
fi
DIR="$(cd "$(dirname "$0")" && pwd)"
mkdir -p ~/.config/systemd/user ~/.cache
cat > "$UNIT" <<UNITEOF
[Unit]
Description=Copy Pico H IMU recordings when the Pico is plugged in
After=graphical-session.target

[Service]
ExecStart=/usr/bin/python3 $DIR/pico_autofetch.py
Restart=always
RestartSec=3
Environment=PATH=$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=default.target
UNITEOF
systemctl --user daemon-reload
systemctl --user enable --now pico-autofetch.service
systemctl --user --no-pager status pico-autofetch.service | head -5
