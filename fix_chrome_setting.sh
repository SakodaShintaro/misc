#!/bin/bash
set -eux

change=${1:-false}

cat /usr/share/applications/google-chrome.desktop | grep Exec


if [ "$change" = true ]; then
    sudo sed -i 's|^Exec=/usr/bin/google-chrome-stable|Exec=/usr/bin/google-chrome-stable --gtk-version=3|' /usr/share/applications/google-chrome.desktop
fi
