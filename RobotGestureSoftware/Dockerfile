FROM ros:kinetic-ros-base
SHELL ["bash", "-c"]

ENV ROS_DISTRO kinetic
ENV WORKSPACE /catkin_ws


### ADD DEPENDENCIES ###

# do dependency installs first to use cache

# get catkin dependencies
# apt-key adv in case cache is being used
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654 \
    && apt-get -q -qq update && apt-get -q -qq install -y \
    ros-${ROS_DISTRO}-catkin \
    python-wstools python-catkin-tools \
    python-tk python-pip \
    bc net-tools iputils-ping

# install for debug purposes
RUN apt-get -q -qq update && apt-get -q -qq install -y \
    less byobu \
    ssh \
    vim \
    emacs

# create and setup aero catkin workspace
RUN mkdir -p ${WORKSPACE}/src/aero-ros-pkg
RUN wstool init ${WORKSPACE}/src
RUN cd ${WORKSPACE}/src/ \
    && git clone https://github.com/seed-solutions/aero-ros-pkg
WORKDIR ${WORKSPACE}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install -q -y -r --from-paths src --ignore-src


### ADD OUR REPO FILES ###

ADD labanotationpy /labanotationpy

# ROS/ includes the following packages:
#     labanotation_msgs
#     labanotation_ros
#     aero_labanotation
#     pepper_labanotation
ADD ROS ${WORKSPACE}/src


### BUILD ###

# install dependencies
WORKDIR ${WORKSPACE}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install -q -y -r --from-paths src --ignore-src \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && catkin config --init

# setup for aero
WORKDIR ${WORKSPACE}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && catkin build aero_description
WORKDIR ${WORKSPACE}/src/aero-ros-pkg/aero_description
RUN source ${WORKSPACE}/devel/setup.bash \
    && ./setup.sh typeFCESy

# build aero
WORKDIR ${WORKSPACE}
RUN source ${WORKSPACE}/devel/setup.bash \
    && catkin build aero_std

# build workspace
WORKDIR ${WORKSPACE}
RUN source ${WORKSPACE}/devel/setup.bash && catkin build


### INSTALL ###

# install labanotationpy
RUN cd /labanotationpy/src && python setup.py install --user

# install dependencies for samples
RUN pip install matplotlib==2.2.3

# install samples
ADD samples /samples
WORKDIR /samples

# create as workspace image
RUN touch /root/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "source ${WORKSPACE}/devel/setup.bash" >> /root/.bashrc
