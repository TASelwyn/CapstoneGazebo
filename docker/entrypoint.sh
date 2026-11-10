#!/usr/bin/env bash
# Container start-up: build the workspace, start the display, then run the
# given command (by default: the simulation).
set -e
source /opt/ros/jazzy/setup.bash
cd /ws

echo "==> Building the workspace..."
colcon build --symlink-install --event-handlers console_cohesion- >/tmp/colcon.log 2>&1 \
  || { cat /tmp/colcon.log; echo "Build failed (log above)."; exit 1; }
source /ws/install/setup.bash

case "${GUI_MODE:-novnc}" in
  novnc)
    # Virtual screen -> VNC -> web page at http://localhost:6080
    export DISPLAY=:1
    export LIBGL_ALWAYS_SOFTWARE=1   # software rendering; no GPU needed
    export QT_X11_NO_MITSHM=1
    Xvfb :1 -screen 0 "${SCREEN_SIZE:-1600x900}x24" +extension GLX +render -noreset >/tmp/xvfb.log 2>&1 &
    sleep 1
    openbox >/tmp/openbox.log 2>&1 &
    x11vnc -display :1 -forever -shared -nopw -quiet -rfbport 5900 >/tmp/x11vnc.log 2>&1 &
    websockify --web /usr/share/novnc 6080 localhost:5900 >/tmp/novnc.log 2>&1 &
    echo "==> Open http://localhost:6080/vnc.html?autoconnect=1&resize=remote in your browser"
    ;;
  x11)
    # Linux only: draw on your own desktop (see compose.x11.yaml).
    echo "==> Using your desktop's display ($DISPLAY)"
    ;;
  none)
    echo "==> No display; pass gui:=false to the launch command."
    ;;
  *)
    echo "Unknown GUI_MODE '$GUI_MODE' (use novnc, x11 or none)"; exit 1 ;;
esac

exec "$@"
