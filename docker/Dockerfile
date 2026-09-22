# Bygger ovanpå PX4:s officiella image (PX4 + Gazebo + ROS 2 + uXRCE-DDS-agent).
# Digest pinnad så att alla i teamet får exakt samma miljö (se StackV2.md, avsnitt 5.2).
FROM px4io/px4-sitl-gazebo-ros2@sha256:2c01c89215a5f541d3a819c1bd2115ff27c9423679f3738dbb4096271d77084b

USER root

# Xvfb = virtuell display att rendera Gazebo mot (containern har ingen egen skärm).
# fluxbox = lättviktig fönsterhanterare, krävs för att Gazebos fönster ska bete sig normalt.
# x11vnc = VNC-server som delar Xvfb-displayen (native VNC-klient, port 5900).
# novnc + websockify = webbläsarbaserad VNC-klient (öppnas via localhost:6080).
RUN apt-get update && apt-get install -y \
    xvfb \
    x11vnc \
    fluxbox \
    novnc \
    websockify \
    && rm -rf /var/lib/apt/lists/*

ENV DISPLAY=:1
ENV VNC_PORT=5900
ENV NOVNC_PORT=6080

COPY start-vnc.sh /usr/local/bin/start-vnc.sh
RUN chmod +x /usr/local/bin/start-vnc.sh

# 5900 = native VNC-klient (t.ex. macOS Skärmdelning, RealVNC Viewer)
# 6080 = noVNC via webbläsare (http://localhost:6080)
EXPOSE 5900 6080

ENTRYPOINT ["/usr/local/bin/start-vnc.sh"]
CMD ["/bin/bash"]
