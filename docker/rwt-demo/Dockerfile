FROM argnctu/jupyter-ros 
MAINTAINER Nick Wang <hchengwang@gmail.com>

# Install system dependencies
RUN apt-get update && apt-get install -y --force-yes tmux wget
RUN apt-get install -y --force-yes ros-kinetic-turtlebot 

# the followings have been installed in argnctu/jupyter-ros
# ros-kinetic-rosbridge-suite ros-kinetic-tf2-web-republisher

RUN apt-get install -y --force-yes ros-kinetic-joint-state-publisher ros-kinetic-visualization-marker-tutorials ros-kinetic-interactive-marker-tutorials ros-kinetic-interactive-marker-proxy ros-kinetic-map-server
RUN apt-get install -y --force-yes vim git python-wstool

# Create userid for building and running ROS applications
RUN useradd -m ros
RUN adduser ros root

# Install map
RUN cd /tmp; wget https://raw.githubusercontent.com/RobotWebTools/rwt_dockers/master/demos/files/map.yaml
RUN mv /tmp/map.yaml /home/ros/
RUN cd /tmp; wget https://raw.githubusercontent.com/RobotWebTools/rwt_dockers/master/demos/files/map.pgm
RUN mv /tmp/map.pgm /home/ros/

# Install turtlebot webview launcher
RUN cd /tmp; wget https://raw.githubusercontent.com/RobotWebTools/rwt_dockers/master/demos/files/rwt_demos.launch
RUN mv /tmp/rwt_demos.launch /home/ros/

# Install turtlebot webview start script
RUN cd /tmp; wget https://raw.githubusercontent.com/RobotWebTools/rwt_dockers/master/demos/files/start.sh
RUN mv /tmp/start.sh /home/ros/
RUN chmod +x /home/ros/start.sh

EXPOSE 9090

CMD ["/bin/bash", "/home/ros/start.sh"]
