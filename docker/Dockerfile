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

# ros_gz_bridge = forwards Gazebo sensor topics (e.g. the lidar point cloud) to ROS 2.
# rviz2 = 3D viewer for ROS 2 data, runs in the VNC desktop (mount-backend.sh lidar).
# python3-opencv = image processing for the camera detector (swarm_backend/detect/).
# python3-scipy = optimal (Hungarian) matching in ByteTrack (swarm_backend/detect/bytetrack.py).
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-rviz2 \
    python3-opencv \
    python3-scipy \
    && rm -rf /var/lib/apt/lists/*

# 3D mapping (hive-vault/mapping/3D mapping plan.md): public ROS packages the DAEP
# exploration planner needs to build and run. DAEP itself is not part of this repo.
# octomap* = the 3D voxel map, octomap_server = builds it from the point cloud,
# irobot_create_msgs = a message DAEP is compiled against, tf_transformations and
# python3-rtree = used by DAEP's Python nodes, octovis = viewer for saved maps (.bt).
RUN apt-get update && apt-get install -y \
    ros-jazzy-octomap \
    ros-jazzy-octomap-msgs \
    ros-jazzy-octomap-ros \
    ros-jazzy-octomap-server \
    ros-jazzy-octovis \
    ros-jazzy-irobot-create-msgs \
    ros-jazzy-tf-transformations \
    python3-rtree \
    && rm -rf /var/lib/apt/lists/*

# ros_gz_bridge pulls in ROS's own gz-transport/gz-msgs "vendor" packages. Sourcing ROS then
# points GZ_CONFIG_PATH at only those, which hides the system Gazebo's `gz sim` command, so
# PX4 (whose entrypoint sources ROS) cannot start Gazebo. ROS prepends to this value, so
# setting it here keeps the system Gazebo commands available.
ENV GZ_CONFIG_PATH=/usr/share/gz

# onnxruntime = runs the trained YOLO model (ONNX) for `--detector yolo`; OpenCV 4.6's own
# ONNX reader cannot run YOLO26. --no-deps so pip leaves the apt numpy and protobuf alone
# (cv2 and Gazebo's Python bindings are built against those); flatbuffers is the one
# dependency it needs at run time.
# On x86 (Stellar) the GPU build, with CUDA 12 and cuDNN 9 from pip (the base image has
# no CUDA): YOLO at 1280 px takes ~9 ms per image on the RTX 3090 Ti instead of ~400 ms on
# the CPU, where three detectors slowed Gazebo to 0.12x real time. detect/yolo_onnx.py
# falls back to the CPU when no GPU is visible (start-stellar.sh without --gpu). Other
# machines (arm64 Macs) keep the CPU build: there is no onnxruntime-gpu for them.
ARG TARGETARCH
RUN if [ "${TARGETARCH:-$(dpkg --print-architecture)}" = "amd64" ]; then \
      pip3 install --break-system-packages --no-deps onnxruntime-gpu==1.22.0 \
        nvidia-cudnn-cu12==9.10.2.21 nvidia-cublas-cu12==12.8.4.1 \
        nvidia-cuda-runtime-cu12==12.8.90 nvidia-cufft-cu12==11.3.3.83 \
        nvidia-curand-cu12==10.3.9.90 nvidia-cuda-nvrtc-cu12==12.8.93 \
        nvidia-nvjitlink-cu12==12.8.93 flatbuffers==24.3.25; \
    else \
      pip3 install --break-system-packages --no-deps onnxruntime==1.20.1 flatbuffers==24.3.25; \
    fi

# Simulation only: the x500 airframe (and every x500 variant, they all source it) sets
# NAV_DLL_ACT=2, "return home when the ground-station link is lost". With that on, PX4
# refuses to arm ("No connection to the GCS") until QGroundControl is connected. Our
# backend arms over ROS 2 and there is no QGC on Stellar, so switch it off (0 = disabled).
# Fails the build if the line is not found, so a changed base image is noticed.
RUN grep -q "^param set-default NAV_DLL_ACT 2$" /opt/px4-gazebo/etc/init.d-posix/airframes/4001_gz_x500 \
    && sed -i 's/^param set-default NAV_DLL_ACT 2$/param set-default NAV_DLL_ACT 0/' /opt/px4-gazebo/etc/init.d-posix/airframes/4001_gz_x500

# PX4 waits only 30 checks (~30 s) for the Gazebo world before giving up with
# "Timed out waiting for Gazebo world". Heavy worlds (hive_base, baylands) can take longer
# on a Mac, so read the count from PX4_GZ_WORLD_TIMEOUT instead (default stays 30).
# Fails the build if the line is not found, so a changed base image is noticed.
RUN grep -q "^ATTEMPTS=30$" /opt/px4-gazebo/etc/init.d-posix/px4-rc.gzsim \
    && sed -i 's/^ATTEMPTS=30$/ATTEMPTS=${PX4_GZ_WORLD_TIMEOUT:-30}/' /opt/px4-gazebo/etc/init.d-posix/px4-rc.gzsim

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
