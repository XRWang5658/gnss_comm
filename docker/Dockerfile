FROM ros:kinetic-perception

# Set environment variables
ENV EIGEN_VERSION="3.3.3"
ENV CATKIN_WS=/root/catkin_ws

# Install all system dependencies
RUN apt-get update && apt-get install -y \
      git \
      cmake \
      python-catkin-tools \
      libgoogle-glog-dev && \
      rm -rf /var/lib/apt/lists/*

# Install Eigen (this dependency installation is fine to keep)
RUN git clone https://gitlab.com/libeigen/eigen.git && \
    cd eigen && \
    git checkout tags/${EIGEN_VERSION} && \
    mkdir build && cd build && \
    cmake .. && make install && \
    cd ../.. && rm -rf eigen

# Create the workspace directory structure
RUN mkdir -p $CATKIN_WS/src

# Set the working directory
WORKDIR $CATKIN_WS

# Add the workspace setup to the entrypoint so every terminal is sourced
RUN sed -i '/exec "$@"/i \
        source "/root/catkin_ws/devel/setup.bash" || true' /ros_entrypoint.sh