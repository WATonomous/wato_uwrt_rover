# Public stand-in for the private ghcr.io/watonomous base images, so the stack builds
# without a GHCR login (e.g. on a personal Mac). Provides the same environment the
# module Dockerfiles rely on: AMENT_WS, WATONOMOUS_INSTALL, rosdep and apt-fast.
FROM ros:humble-ros-base

# Outside /root: the module Dockerfiles clean up with `rm -rf /root/*`.
ENV AMENT_WS=/ament_ws
ENV WATONOMOUS_INSTALL=/opt/watonomous

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# apt-fast is only a download accelerator in the WATO images; plain apt-get behaves the same.
RUN printf '#!/bin/sh\nexec apt-get "$@"\n' > /usr/local/bin/apt-fast && \
    chmod +x /usr/local/bin/apt-fast && \
    mkdir -p "${AMENT_WS}/src" "${WATONOMOUS_INSTALL}" && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" > "${WATONOMOUS_INSTALL}/setup.bash"

WORKDIR ${AMENT_WS}
