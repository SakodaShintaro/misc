#!/bin/bash
set -eux

cat /usr/share/applications/google-chrome.desktop | grep Exec
# sudo sed -i 's|^Exec=/usr/bin/google-chrome-stable|Exec=/usr/bin/google-chrome-stable --gtk-version=3|' /usr/share/applications/google-chrome.desktop
