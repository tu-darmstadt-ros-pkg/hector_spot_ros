FROM ros:melodic-ros-base
SHELL ["/bin/bash", "-c"]

ENV SDK_VERSION=3.2.3
# Install dependencies
RUN apt-get update && apt-get install -y python3-pip python-catkin-tools
RUN pip3 install --upgrade pip

# Install Python3 ROS dependencies
RUN python3 -m pip install rospkg catkin_pkg opencv-python

# Set up workspace
RUN mkdir -p catkin_ws/src
WORKDIR /catkin_ws
RUN source /opt/ros/melodic/setup.bash && catkin init
#RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc
RUN sed -i 's#/opt/ros/$ROS_DISTRO/setup.bash#/catkin_ws/devel/setup.bash#' /ros_entrypoint.sh

# Install bosdyn client
RUN python3 -m pip install bosdyn-client==$SDK_VERSION

# Install dependencies
WORKDIR /catkin_ws/src
RUN git clone https://github.com/tu-darmstadt-ros-pkg/move_base_lite.git --branch hector_exploration
RUN touch /catkin_ws/src/move_base_lite/move_base_lite_server/CATKIN_IGNORE

#RUN git clone https://git.sim.informatik.tu-darmstadt.de/hector/hector_spot_ros.git --branch master
# Copy hector_spot_ros
RUN mkdir hector_spot_ros
COPY . hector_spot_ros/
COPY wait_for_roscore.sh /

# Pull dependencies
RUN apt update && rosdep install --from-paths . --ignore-src -r -y

# Environment setup
RUN source /opt/ros/melodic/setup.bash && catkin build
RUN echo 'source /catkin_ws/devel/setup.bash' >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["roslaunch", "hector_spot_ros", "spot.launch"]
