FROM ros:jazzy-ros-base AS common
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=42
ENV ROS_LOCALHOST_ONLY=0

FROM common AS simulator
# Webots version is pinned together with webots_ros2 2025.0.x, which targets R2025a.
ARG WEBOTS_VERSION=R2025a
ARG WEBOTS_SHA256=c5127fb4206c57a5ae5523f1b7f3da8b670bc8926d9ae08595e139f226f38c38
# Runtime libraries follow Webots scripts/install/linux_runtime_dependencies.sh
# (without ffmpeg/g++, which are only needed for movie export and native controllers).
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-webots-ros2-driver python3-pil \
    ca-certificates wget bzip2 xvfb xauth \
    libgl1-mesa-dri libglu1-mesa libegl1 libxkbcommon-x11-0 libxcb-cursor0 \
    libxcb-keysyms1 libxcb-image0 libxcb-icccm4 libxcb-randr0 libxcb-render-util0 \
    libxcb-xinerama0 libxcb-shape0 libxcomposite1 libxtst6 libnss3 libsndio7.0 \
    && rm -rf /var/lib/apt/lists/* \
    && wget -q "https://github.com/cyberbotics/webots/releases/download/${WEBOTS_VERSION}/webots-${WEBOTS_VERSION}-x86-64.tar.bz2" -O /tmp/webots.tar.bz2 \
    && echo "${WEBOTS_SHA256}  /tmp/webots.tar.bz2" | sha256sum -c - \
    && tar -xjf /tmp/webots.tar.bz2 -C /usr/local \
    && rm /tmp/webots.tar.bz2
ENV WEBOTS_HOME=/usr/local/webots
# Use X11 (Xvfb headless, XWayland/Xorg for the window); the bundled Wayland plugin lacks libraries.
ENV QT_QPA_PLATFORM=xcb
COPY simulation /opt/capstone/simulation
COPY docker/start_simulator.sh /opt/capstone/start_simulator.sh
# Webots warns when started as root; the base image already has the unprivileged "ubuntu" user.
USER ubuntu
ENV USER=ubuntu
CMD ["bash", "/opt/capstone/start_simulator.sh"]

# Only package.xml files: the dependency layer below is rebuilt when dependencies change, not code.
FROM common AS manifests
COPY ros2_ws/src /tmp/src
RUN mkdir -p /tmp/manifests \
    && cd /tmp/src && find . -name package.xml -exec cp --parents {} /tmp/manifests \;

# Development environment: compilers, tools and every rosdep dependency declared in package.xml.
# `make shell/build/test` run this image with the repository mounted at /workspace.
FROM common AS dev
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ccache gdb python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
COPY --from=manifests /tmp/manifests /tmp/manifests
RUN apt-get update \
    && rosdep update --rosdistro "$ROS_DISTRO" \
    && rosdep install --from-paths /tmp/manifests --ignore-src --rosdistro "$ROS_DISTRO" -y \
    && rm -rf /var/lib/apt/lists/* /tmp/manifests
COPY docker/dev_env.sh /etc/capstone_dev.sh
RUN echo "source /etc/capstone_dev.sh" >> /etc/bash.bashrc

# Submission/CI image: the same environment with the workspace built and tested inside.
FROM dev AS autonomy
WORKDIR /ws
COPY ros2_ws/src /ws/src
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && colcon test --event-handlers console_direct+ \
    && colcon test-result --verbose
CMD ["bash", "-c", "source /ws/install/setup.bash && exec ros2 run capstone_bringup clock_probe"]
