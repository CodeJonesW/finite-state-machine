FROM ros:humble

# Install necessary Python build tools
RUN apt update && apt install -y python3-colcon-common-extensions python3-pip

# Set up workspace
WORKDIR /ros2_ws

# Copy source code
COPY ./src ./src

# Source ROS environment and build the Python package correctly
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select fsm_demo


CMD ["/bin/bash"]
