FROM ubuntu:18.04
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

ENV DEBIAN_FRONTEND=noninteractive 
ENV FLIGHTMARE_PATH=/home/flightmare 

# Installing some essential system packages
RUN apt-get update && apt-get install -y --no-install-recommends \
   lsb-release \
   build-essential \
   python3 python3-dev python3-pip \
   cmake \
   git \
   vim \
   ca-certificates \
   libzmqpp-dev \
   libopencv-dev \
   gnupg2 \
   curl\
   && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip

RUN /bin/bash -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Installing ROS  Melodic
RUN apt-get update && apt-get install -y --no-install-recommends \
   ros-melodic-desktop-full 

# Installing catkin tools
RUN apt-get update && apt-get install -y python3-setuptools && pip3 install catkin-tools 

RUN cd /home && git clone https://github.com/ongdexter/flightmare.git flightmare-ros

RUN cd /home/flightmare-ros/flightlib && pip3 install . \
    && cd /home/flightmare-ros/flightrl && pip3 install . 

# Installing with pip
RUN cd /home && git clone https://github.com/ongdexter/flightmare.git flightmare-pip \
    && pip3 install tensorflow-gpu==1.14 \
    && pip3 install scikit-build \
    && cd /home/flightmare-pip/flightrl && pip3 install .