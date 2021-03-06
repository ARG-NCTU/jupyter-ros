FROM osrf/ros:kinetic-desktop-xenial

SHELL ["/bin/bash","-c"]

# install ros packages
RUN apt-get update && apt-get install -y \
    apt-utils \
    ros-kinetic-desktop-full=1.3.2-0* \
    && rm -rf /var/lib/apt/lists/*

ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && apt-get update \
    && cd $CATKIN_WS \
    && catkin_make

RUN apt-get update && apt-get install -y python-pip
RUN pip install pip==9.0.1
RUN apt-get -y install curl
RUN curl -sL https://deb.nodesource.com/setup_11.x | bash
RUN apt-get -y install nodejs

RUN pip install jupyter ipywidgets jupyterlab bqplot pyyaml jupyros numpy==1.12.0
RUN jupyter labextension install @jupyter-widgets/jupyterlab-manager
RUN jupyter labextension install jupyter-ros
RUN jupyter nbextension enable --py --sys-prefix jupyros
RUN jupyter nbextension enable --py --sys-prefix widgetsnbextension

# install packages for notebook examples
RUN pip install sidecar
RUN jupyter labextension install @jupyter-widgets/jupyterlab-sidecar

RUN apt-get install -y ros-${ROS_DISTRO}-teb-local-planner ros-${ROS_DISTRO}-interactive-marker-proxy ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-rosbridge-server
RUN git clone https://github.com/RobotWebTools/tf2_web_republisher /root/catkin_ws/src/tf2_web_republisher/

WORKDIR $CATKIN_WS/src
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd $CATKIN_WS \
    && catkin_make

RUN git clone https://github.com/ARG-NCTU/jupyter-ros.git /root/jupyter-ros/

EXPOSE 8888

COPY ./ros_catkin_entrypoint.sh /
ENTRYPOINT ["/ros_catkin_entrypoint.sh"]
CMD ["bash"]
