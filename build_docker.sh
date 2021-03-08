#!/bin/bash

# build the simulation docker 
docker build -t simulator -f docker/SimulatorDockerfile_GPU .
#docker build -t simulatorheadless -f docker/SimulatorDockerfileHeadless .
#build the rtreach docker
docker build -t rtreach -f docker/Dockerfile .
