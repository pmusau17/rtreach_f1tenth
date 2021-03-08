#! /bin/bash

#run the docker container
docker container run -it --name=rtreach_ntainer  --rm --net=host rtreach /bin/bash -c "source devel/setup.bash && roslaunch rtreach multi_agent_reach.launch"