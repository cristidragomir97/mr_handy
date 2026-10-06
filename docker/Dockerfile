# syntax=docker/dockerfile:1
FROM ros:jazzy-ros-base-noble
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions python3-venv python3-pip python3-numpy \
    python3-tornado python3-pil python3-psutil python3-yaml \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-mujoco-ros2-control \
    ros-jazzy-navigation2 ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
    ros-jazzy-laser-filters ros-jazzy-rviz2 \
    ros-jazzy-twist-mux ros-jazzy-mujoco-ros2-control-plugins \
    ros-jazzy-rmw-zenoh-cpp ros-jazzy-robot-state-publisher ros-jazzy-xacro \
    libegl-mesa0 libgl1-mesa-dri libglx-mesa0 mesa-utils \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv --system-site-packages /opt/handy101-checks \
    && /opt/handy101-checks/bin/pip install --no-cache-dir mujoco==3.6.0 scipy pillow
# BuildKit manages these GitHub contexts and resolves their default-branch HEAD
# on every image build. Keep dependency sources available for container rebuilds.
COPY --from=base101 /src/base101/base101_description /opt/handy101_dependencies/src/base101_description
COPY --from=base101 /src/base101/base101_control /opt/handy101_dependencies/src/base101_control
COPY --from=base101 /src/base101/base101_slam /opt/handy101_dependencies/src/base101_slam
COPY --from=base101 /src/base101/base101_nav /opt/handy101_dependencies/src/base101_nav
COPY --from=base101 /src/base101/base101_lidar/config/lidar_filters.yaml /opt/handy101_dependencies/src/base101_nav/config/handy101_lidar_filters.yaml
COPY --from=base101 /src/rosboard /opt/handy101_dependencies/src/rosboard
COPY --from=mod101 /src/mod101_description /opt/handy101_dependencies/src/mod101_description
COPY --from=mod101 /src/mod101_tool_jaws /opt/handy101_dependencies/src/mod101_tool_jaws
COPY --from=mod101 /src/mod101_tool_none /opt/handy101_dependencies/src/mod101_tool_none
COPY --from=mod101 /src/mod101_tool_parallel /opt/handy101_dependencies/src/mod101_tool_parallel
COPY --from=mod101 /src/mod101_tool_pincopen /opt/handy101_dependencies/src/mod101_tool_pincopen
COPY --from=mod101 /src/mod101_tool_camera /opt/handy101_dependencies/src/mod101_tool_camera
COPY --from=mod101 /src/mod101_tool_pggripper /opt/handy101_dependencies/src/mod101_tool_pggripper

COPY docker/prepare_dependencies.py /opt/handy101_dependencies/prepare.py
RUN python3 /opt/handy101_dependencies/prepare.py

COPY assets/robocasa /opt/handy101-assets/robocasa
ENV HANDY101_WORLD_ASSETS=/opt/handy101-assets/robocasa
COPY src /opt/handy101_ws/src/handy101
RUN source /opt/ros/jazzy/setup.bash && cd /opt/handy101_ws \
    && colcon build --symlink-install --parallel-workers 2 \
       --base-paths src /opt/handy101_dependencies/src \
       --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
    && source install/setup.bash \
    && for backend in mock mujoco; do \
         xacro install/handy101_description/share/handy101_description/urdf/handy101.xacro \
           control_backend:=$backend > /dev/null || exit 1; \
       done
COPY docker/entrypoint.sh /usr/local/bin/handy101-entrypoint
RUN chmod +x /usr/local/bin/handy101-entrypoint
ENV ROS_DOMAIN_ID=184 RMW_IMPLEMENTATION=rmw_zenoh_cpp
WORKDIR /workspace/handy101
ENTRYPOINT ["/usr/local/bin/handy101-entrypoint"]
CMD ["sleep", "infinity"]
