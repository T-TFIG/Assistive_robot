FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y \
    ros-humble-nav2-rviz-plugins \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-twist-mux \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-generate-parameter-library \
    ros-humble-control-toolbox \
    ros-humble-realtime-tools \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-simulations \
    ros-humble-turtlebot3-gazebo \
    ros-humble-nav2-rviz-plugins \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    && rm -rf /var/lib/apt/list/*

# make our workspace
WORKDIR /Assistive_robot
COPY ./src ./src


SHELL ["/bin/bash", "-c"]

# install from our package.xml
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -y

# source 
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --merge-install

# written down inside the bash terminal (no more source after this one)
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /Assistive_robot/install/setup.bash" >> ~/.bashrc

ENTRYPOINT [ "/bin/bash" ]