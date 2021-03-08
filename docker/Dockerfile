FROM ros:kinetic-robot
RUN apt-get update && apt-get install apt-transport-https 
RUN apt-get update &&  apt-get install -y ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control ros-kinetic-ackermann-msgs ros-kinetic-joy
RUN apt-get update &&  apt-get install -y ros-kinetic-teb-local-planner ros-kinetic-move-base ros-kinetic-navigation ros-kinetic-hector-slam ros-kinetic-driver-common ros-kinetic-actionlib


#install pip
RUN apt-get install -y python-pip && apt-get install -y python3-pip

RUN pip install rospkg defusedxml PySide2
RUN pip install empy 

#Need these packages for debugging
RUN apt-get install -y nano
RUN apt-get install -y net-tools


# bootstrap rosdep
RUN rosdep update

# clone the repository
RUN git clone https://github.com/pmusau17/rtreach_f1tenth.git 
WORKDIR rtreach_f1tenth/
RUN /bin/bash -c "git pull && source /opt/ros/kinetic/setup.bash && ./build_rtreach.sh"

WORKDIR .. 


WORKDIR rtreach_ros/
RUN ls src/rtreach/launch
CMD /bin/bash -c "source devel/setup.bash && rosrun rtreach reach_node porto_obstacles.txt"

