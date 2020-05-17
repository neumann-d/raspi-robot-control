# Prerequisites
- Install docker (version >=18) and add your user to the docker group, see also:
https://docs.docker.com/install/linux/linux-postinstall/

# Build

- Build docker image

```
./build.sh
```

- XBox Controller Setup: In order to use a connected XBox Controller inside this docker container you have to set udev rules on your machine (needs root rights)

```
echo 'SUBSYSTEM=="usb",GROUP="plugdev",MODE="0664"' | sudo tee -a /etc/udev/rules.d/90-usbpermission.rules
```


# Run

- Run docker container. A workspace directory on the host computer should be mounted in the container to enable persistent changes (can be adjusted in `run.sh` file, uses `../ros-robot-control` by default).

```
./run.sh    # or use ./run-nvidia.sh when host is running nvidia driver (needs nvidia-container-toolkit installed on host)
```

- Compile mounted workspace sources (in the running docker container).

```
cd /root/workspace/
catkin build
```

# Update docker image (optional)

Can apply additional changes to the ros-docker image with `docker commit`. Don't close the current terminal, where you run the docker container. Open a second terminal and run:

```
docker commit ros-docker ros-docker:latest
docker images
```

# Backup and restore image (optional)

If you want to backup your local installed docker image (to avoid downloading everything again on antoher computer), you can do the following:

- Save installed docker image

```
docker image save ros:melodic | gzip -c > ros-melodic.tar.gz
docker image save ros-docker:latest | gzip -c > ros-docker.tar.gz
```

- Restore image into your docker installation

```
gunzip -c ros-melodic.tar.gz | docker load
gunzip -c ros-docker.tar.gz | docker load
```

