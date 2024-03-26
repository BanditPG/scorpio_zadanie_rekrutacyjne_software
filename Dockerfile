FROM ros:noetic

RUN apt-get update && apt-get install -y
RUN apt-get install ros-noetic-rosbridge-suite -y

RUN mkdir -p /opt/ros/catkin_ws/src
WORKDIR /opt/ros/catkin_ws
COPY virtual_dc_motor ./src/virtual_dc_motor
COPY start.sh ./

RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

CMD ["./start.sh"]