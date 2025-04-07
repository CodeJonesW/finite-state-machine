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


# Install your Python package
RUN pip install -e /ros2_ws/src/fsm_demo

# Auto-source ROS + workspace every time you enter the container
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
