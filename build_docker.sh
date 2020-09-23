#!/bin/bash

# build the simulation docker 
docker build -t simulator -f docker/SimulatorDockerfile_GPU .

# build the rtreach docker
docker build -t rtreach -f docker/Dockerfile .