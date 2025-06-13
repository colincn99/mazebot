FROM osrf/ros:noetic-desktop-full-focal

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-turtlebot3-description \
    ros-noetic-turtlebot3-gazebo \
    ros-noetic-turtlebot3-slam \
    ros-noetic-gmapping \
    ros-noetic-move-base \
    ros-noetic-dwa-local-planner \
    # python3-pip \
    && rm -rf /var/lib/apt/lists/*
#
# RUN pip3 install pyquaternion
RUN adduser user
COPY . /home/user/catkin_ws/src/mazebot
WORKDIR /home/user/catkin_ws
SHELL ["/bin/bash", "-ec"]
RUN source /opt/ros/noetic/setup.bash && catkin_make
RUN chown -R user /home/user

WORKDIR /home/user
USER user 
CMD ["/bin/bash", "-ec", "source /home/user/catkin_ws/devel/setup.bash && \
      roslaunch mazebot run_with_planner.launch & \
      sleep 5 && \
      source /home/user/catkin_ws/devel/setup.bash && \
      rosrun mazebot wall_explore.py"]
