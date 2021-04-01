#!/bin/bash

pushd src

# Compile the source files needed  to create the C shared library files
gcc -c -std=gnu99 -O3 -Wall  -fpic face_lift_bicycle_model.c geometry.c interval.c simulate_bicycle.c util.c  dynamics_bicycle_model.c bicycle_safety.c bicycle_model.c face_lift_bicycle_model_visualization.c bicycle_model_vis.c bicycle_dynamic_safety.c bicycle_model_dynamic_vis.c -lm
gcc -c -std=gnu99 -Wall -fpic face_lift_parametrizeable.c geometry.c interval.c util.c simulate_bicycle.c dynamics_bicycle_model.c bicycle_model_parametrizeable.c -lm 

# There are five shared library files we need
gcc -shared -o libRtreachdyn.so face_lift_parametrizeable.o dynamics_bicycle_model.o geometry.o interval.o util.o simulate_bicycle.o bicycle_model_parametrizeable.o
gcc -shared -o libRtreach.so face_lift_bicycle_model.o bicycle_model.o dynamics_bicycle_model.o geometry.o interval.o  simulate_bicycle.o util.o bicycle_safety.o 
gcc -shared -o libRtreachvis.so face_lift_bicycle_model_visualization.o bicycle_model_vis.o dynamics_bicycle_model.o geometry.o interval.o  simulate_bicycle.o util.o bicycle_safety.o
gcc -shared -o libRtreachDynamicvis.so face_lift_bicycle_model_visualization.o bicycle_model_dynamic_vis.o dynamics_bicycle_model.o geometry.o interval.o  simulate_bicycle.o util.o bicycle_dynamic_safety.o
gcc -c -std=gnu99 -fpic face_lift_obstacle_visualization.c geometry.c interval.c util.c  simulate_obstacle.c  dynamics_obstacle.c  obstacle_model_plots.c -lm -DOBSTACLE_MODEL
gcc -shared -o libRtreachObs.so face_lift_obstacle_visualization.o dynamics_obstacle.o geometry.o interval.o  simulate_obstacle.o util.o obstacle_model_plots.o
popd 

# create the rtreach rospkg
mkdir -p ../rtreach_ros/src
# copy the rospkg into the newly created directory
cp -r ros_src/rtreach/ ../rtreach_ros/src/

# copy the batch files into the rospkg
cp run_batch_rl.sh run_batch.sh reproduce_emsoft_experiments.sh run_emsoft_experiments.sh run_batch_worlds.sh ../rtreach_ros

# copy all the neccessary header files and library files into the rospackage
cp src/libRtreachDynamicvis.so  src/libRtreachdyn.so  src/libRtreachObs.so  src/libRtreach.so  src/libRtreachvis.so src/bicycle_dynamic_safety.h  src/bicycle_model_parametrizeable.h  src/bicycle_safety.h  src/dynamics_bicycle.h  src/dynamics_obstacle.h  src/geometry.h  src/main.h src/face_lift.h src/simulate_bicycle_plots.h  src/simulate_obstacle.h ../rtreach_ros/src/rtreach/src/

pushd ../rtreach_ros
catkin_make
popd