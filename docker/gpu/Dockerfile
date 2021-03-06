FROM nvidia/cuda:9.0-cudnn7-devel-ubuntu16.04
#FROM nvidia/cuda:9.0-cudnn7-devel-ubuntu16.04
#FROM osrf/ros:kinetic-desktop-xenial

SHELL ["/bin/bash","-c"]

# Prerequisite linux tool for ROS 
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


# ROS workspace in container
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && apt-get update \
    && cd $CATKIN_WS \
    && catkin_make


# Jupyter installation
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

# Pytorch installation: cuda 9.0 + python 2.7 + pip
RUN pip install torch==1.0.1 -f https://download.pytorch.org/whl/cu90/stable torchvision typing
# Pytorch installation: cuda 10.0 + python 2.7 + pip
# RUN pip install --no-cache-dir torch torchvision

# packages to run examples
RUN pip install sidecar
RUN jupyter labextension install @jupyter-widgets/jupyterlab-sidecar

RUN apt-get install -y ros-${ROS_DISTRO}-teb-local-planner ros-${ROS_DISTRO}-interactive-marker-proxy ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-rosbridge-server
RUN git clone https://github.com/RobotWebTools/tf2_web_republisher /root/catkin_ws/src/tf2_web_republisher/

WORKDIR $CATKIN_WS/src
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd $CATKIN_WS \
    && catkin_make
#RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd /root/catkin_ws; catkin_make'

RUN git clone https://github.com/ARG-NCTU/jupyter-ros.git /root/jupyter-ros/

EXPOSE 8888

COPY ./ros_catkin_entrypoint.sh /
ENTRYPOINT ["/ros_catkin_entrypoint.sh"]
CMD ["bash"]
