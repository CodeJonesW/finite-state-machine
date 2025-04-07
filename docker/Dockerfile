FROM ros:humble

# Install Python + colcon tools
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    && apt clean

# Setup working dir
WORKDIR /ros2_ws

# Copy source code
COPY ./src ./src

# Source ROS and build workspace (install gets generated)
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select fsm_demo"

# Install your package in editable mode AFTER build
RUN pip install -e /ros2_ws/src/fsm_demo

# Auto-source ROS and workspace on container start
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source /ros2_ws/install/setup.bash' >> ~/.bashrc

# Default command
CMD ["/bin/bash"]
