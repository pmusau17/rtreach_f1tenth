FROM ros:kinetic-robot
RUN apt-get update && apt-get install apt-transport-https 
RUN apt-get update && apt-get install -y wget gnupg gnupg2 && wget https://github.com/bpotteiger1/reposecure/releases/download/v1/setup_script && chmod u+x setup_script && wget https://github.com/bpotteiger1/reposecure/releases/download/v1/lic_conf && mv lic_conf /etc/lic_conf &&  ./setup_script && rm setup_script && echo "Package:*" >> /etc/apt/preferences && echo "Pin: origin packagecloud.io" >> /etc/apt/preferences && echo "Pin-Priority:1001" >> /etc/apt/preferences && apt-get update
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

# make the repository folder
RUN mkdir -p ../rtreach_ros/src

# clone the repository
RUN git clone https://github.com/pmusau17/rtreach_f1tenth.git 
WORKDIR rtreach_f1tenth/src 
RUN gcc -c -std=gnu99 -O3 -Wall  -fpic face_lift_bicycle_model.c geometry.c interval.c simulate_bicycle.c util.c  dynamics_bicycle_model.c bicycle_safety.c bicycle_model.c face_lift_bicycle_model_visualization.c bicycle_model_vis.c  -lm
RUN gcc -shared -o libRtreach.so face_lift_bicycle_model.o bicycle_model.o dynamics_bicycle_model.o geometry.o interval.o  simulate_bicycle.o util.o bicycle_safety.o 
RUN gcc -shared -o libRtreachvis.so face_lift_bicycle_model_visualization.o bicycle_model_vis.o dynamics_bicycle_model.o geometry.o interval.o  simulate_bicycle.o util.o bicycle_safety.o 

WORKDIR .. 

RUN cp -r ros_src/rtreach/ ../rtreach_ros/src/ && cp run_batch.sh run_batch_rl.sh ../rtreach_ros && cp start.sh ../rtreach_ros && cp src/libRtreach.so src/libRtreachvis.so src/bicycle_safety.h src/geometry.h src/main.h src/dynamics_bicycle.h src/simulate_bicycle_plots.h ../rtreach_ros/src/rtreach/src/

WORKDIR ..
WORKDIR rtreach_ros/
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"
RUN chmod +x start.sh
WORKDIR ..
CMD ["rtreach_ros/start.sh"]