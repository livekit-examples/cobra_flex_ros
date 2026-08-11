FROM ros:jazzy

WORKDIR /cobra_flex_ros

# Workspace sources. src/externals/ is gitignored, so fetch it with vcs when
# the build context doesn't already carry a checkout.
COPY cobra_flex.repos .
COPY src src
RUN [ -d src/externals/topic_tools ] \
    || (mkdir -p src/externals && vcs import src/externals < cobra_flex.repos)

# Install every dependency the package manifests declare (python3-serial &c).
RUN apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO}" \
    && rm -rf /var/lib/apt/lists/*

RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && colcon build --packages-up-to cobra_flex_bringup

# Pre-source the overlay for interactive shells (docker exec -it cobra bash).
# NOTE: the dev compose service bind-mounts the host workspace over
# /cobra_flex_ros, shadowing the build baked in here; run the image without
# that volume to use the self-contained build.
RUN echo 'source /cobra_flex_ros/install/setup.bash' >> /root/.bashrc
