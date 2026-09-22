#!/usr/bin/env bash
set -e

# Starta virtuell display (1920x1080 matchar ett vanligt browserfönster bättre än 1280x720,
# så noVNC behöver skala mindre för att fylla ytan)
Xvfb "${DISPLAY}" -screen 0 1920x1080x24 &
sleep 1

# Starta fönsterhanterare mot den virtuella displayen
fluxbox &

# Dela displayen över VNC (native klient, port 5900). macOS Skärmdelning hanterar
# "None"-autentisering opålitligt (hänger på "Ansluter..."), så vi sätter ett lösenord.
VNC_PASSWORD="${VNC_PASSWORD:-hive1234}"
x11vnc -display "${DISPLAY}" -forever -shared -passwd "${VNC_PASSWORD}" -rfbport "${VNC_PORT}" -bg -o /var/log/x11vnc.log

# Dela samma VNC-server via webbläsaren (noVNC, port 6080)
websockify --web=/usr/share/novnc "${NOVNC_PORT}" "localhost:${VNC_PORT}" &

# Kör vidare in i basimagens vanliga entrypoint (samma som utan VNC-lagret,
# se avsnitt 5.3 i StackV2.md), t.ex. MicroXRCEAgent eller px4-gazebo.
exec /usr/local/bin/ros2-entrypoint.sh "$@"
