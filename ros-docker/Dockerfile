### the ros image
FROM ros:melodic

# install ssh, wget and other useful tools
RUN apt-get update && apt-get install -y \
    apt-transport-https \
    bash-completion \
    ccache \
    nano \
    wget \
    ssh \
    terminator \
    nemo \
    dbus-x11 \
    xboxdrv \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# install rosdeps and catkin tools
RUN apt-get update && apt-get install -y \
    python-catkin-lint \
    python-catkin-tools

# set up ccache
RUN /usr/sbin/update-ccache-symlinks
RUN echo 'export PATH="/usr/lib/ccache:$PATH"' | tee -a /root/.bashrc

# fix auto completion
RUN rm /etc/apt/apt.conf.d/docker-clean
RUN echo 'source /etc/bash_completion' | tee -a /root/.bashrc
RUN apt-get update

# install ros-melodic-desktop-full (full ros tools including RViz, rqt etc.)
RUN apt-get install -y ros-melodic-desktop-full

# copy entrypoint script (which is sourcing ros setup.bash)
COPY entrypoint.sh entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh"]
CMD ["/bin/bash"]

