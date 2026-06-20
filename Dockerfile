ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO=jazzy
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    git \
    mesa-utils \
    python3-colcon-common-extensions \
    python3-opencv \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt-image-view \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-xacro \
    sudo \
    && rm -rf /var/lib/apt/lists/*

RUN if getent group "${USER_GID}" >/dev/null; then \
      EXISTING_GROUP="$(getent group "${USER_GID}" | cut -d: -f1)"; \
      groupmod -n "${USERNAME}" "${EXISTING_GROUP}" || true; \
    else \
      groupadd --gid "${USER_GID}" "${USERNAME}"; \
    fi \
    && if id -u "${USER_UID}" >/dev/null 2>&1; then \
      EXISTING_USER="$(getent passwd "${USER_UID}" | cut -d: -f1)"; \
      usermod -l "${USERNAME}" -d "/home/${USERNAME}" -m "${EXISTING_USER}" || true; \
    else \
      useradd --uid "${USER_UID}" --gid "${USER_GID}" -m "${USERNAME}"; \
    fi \
    && usermod -aG sudo,video,render "${USERNAME}" || true \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

RUN rosdep init 2>/dev/null || true

USER ${USERNAME}
WORKDIR /home/${USERNAME}/drone-aruco-docking/workspace

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "/home/${USERNAME}/.bashrc" \
    && echo 'if [ -f /home/${USER}/drone-aruco-docking/workspace/install/setup.bash ]; then source /home/${USER}/drone-aruco-docking/workspace/install/setup.bash; fi' >> "/home/${USERNAME}/.bashrc"

CMD ["bash"]
