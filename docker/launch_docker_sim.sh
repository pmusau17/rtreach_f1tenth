#! /bin/bash

#run the docker container
docker container run --runtime=nvidia -it -e DISPLAY  --rm --net=host --env="QT_X11_NO_MITSHM=1" -v /tmp/.X11-unix:/tmp/.X11-unix simulator /bin/bash -c "source devel/setup.bash && roslaunch race sim_for_rtreach_multi_agent.launch number_of_cars:=3 docker:=true"