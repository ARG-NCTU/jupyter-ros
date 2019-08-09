FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu16.04
# FROM osrf/ros:kinetic-desktop-xenial

SHELL ["/bin/bash","-c"]

# Prerequisite for ROS 
RUN apt-get update -qq > /dev/null && apt-get install -y -qq sudo wget lsb-release iputils-ping > /dev/null && \
    apt-get install -y -qq build-essential vim htop sshfs nfs-common git && \
    rm -rf /var/lib/apt/lists/*

# ROS installation: kinetic
ENV ROS_DISTRO kinetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && apt-get install -y \
    apt-utils \
    ros-${ROS_DISTRO}-desktop-full=1.3.2-0* && \
    rosdep init && \
    rosdep update && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    source /root/.bashrc

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
RUN git clone https://github.com/RoboStack/jupyter-ros.git /root/jupyter-ros/

# Pytorch installation: cuda 10.0 + python 2.7 + pip
RUN pip install --no-cache-dir torch torchvision


EXPOSE 8888

COPY ./ros_catkin_entrypoint.sh /
ENTRYPOINT ["/ros_catkin_entrypoint.sh"]
CMD ["bash"]
