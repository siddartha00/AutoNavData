FROM osrf/ros:jazzy-desktop-full

# Install TurtleBot 4 and Simulation dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-turtlebot4-desktop \
    ros-jazzy-turtlebot4-simulator \
    ros-jazzy-turtlebot4-gz-bringup \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace
WORKDIR /ros2_ws
RUN mkdir src

# Automatically source ROS and your workspace in every new terminal
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]