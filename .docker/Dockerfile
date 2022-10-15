ARG ROS_DISTRO="melodic"
FROM osrf/ros:${ROS_DISTRO}-desktop
ENV OVERLAY_WS /root/overlay_ws
WORKDIR $OVERLAY_WS/src

# Copy source files
COPY . pcl_tutorials_ros

# Install depnedencies
# wget is required to download pcd files.
# The add-apt-repository command needs software-properties-common package.
# https://github.com/IntelRealSense/librealsense/blob/c94410a420b74e5fb6a414bd12215c05ddd82b69/doc/distribution_linux.md
RUN apt-get update && \
    apt-get install -y software-properties-common git && \
    apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
    add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u && \
    apt-get install -y \
        librealsense2-dev \
        librealsense2-utils \
        librealsense2 \
        librealsense2-gl \
        librealsense2-net \
        librealsense2-udev-rules && \
    rosdep update && \
    rosdep install -iy --from-paths . && \
    apt-get install wget && \
    cd pcl_tutorials_ros/samples && ./download.sh && \
    rm -rf /var/lib/apt/lists/

# Build package
ARG CATKIN_OPTION=""
RUN cd $OVERLAY_WS && \
        . /opt/ros/${ROS_DISTRO}/setup.sh && \
        catkin_make ${CATKIN_OPTION}

# Download gazebo models by sparse checkout
RUN mkdir -p /root/.gazebo/models && \
    cd  /root/.gazebo/models && \
    git init . && \
    git config core.sparsecheckout true && \
    echo "ground_plane" >> .git/info/sparse-checkout && \
    echo "sun" >> .git/info/sparse-checkout && \
    echo "table" >> .git/info/sparse-checkout && \
    git remote add origin https://github.com/osrf/gazebo_models && \
    git pull origin master && \
    rm -rf .git

# Edit entrypoint to source overlay setup file
WORKDIR $OVERLAY_WS
RUN sed --in-place --expression \
    '$i if [ -e $OVERLAY_WS/devel/setup.bash ]; then\n\tsource "$OVERLAY_WS/devel/setup.bash" \nfi' \
    /ros_entrypoint.sh
