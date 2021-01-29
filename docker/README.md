# Docker Development Image Setup Guide

## You have to read this
- Run `docker-build.sh` to build the development image.
- Run `docker-run.sh` to start the container.
- Run `docker-join.sh` to join the currently running container.
- If you add a ROS dependency or edit `Dockerfile.local`, re-run `docker-build.sh`.
- If you edit `Dockerfile`, run `docker-build-base.sh`, then `docker-build.sh`.

## You don't have to read this

### Remote Image
`Dockerfile` is used to build `purduearc/rocket_league:latest`.
[DockerHub](https://hub.docker.com/repository/docker/purduearc/rocket-league/)
is set up to automatically build this image when changes are made to the `main`
branch to this repository. You can build this image locally by running
`docker-build-base.sh`. You might do this if you make any changes to
`Dockerfile`, such as adding a system library from `apt` or if you need to
install anything from source. This image uses a generic user and does not
include any code from this repository or install any ROS dependencies other than
those found in the standard `ros-melodic-desktop-full` install.

### Local Image
`Dockerfile.local` is used to build `purduearc/rocket_league:local`. DockerHub
will not automatically build this image, you have to do that yourself by running
`docker-build.sh`. Running this will install all ROS dependencies in the image
and add your user so files can be accessed and modified from both the container
and your computer.

### Running the image
Run `docker-run.sh` to start the image (`purduearc/rocket_league:local`) and
mount your local catkin workspace. If you have an NVidia GPU, you can use the
`--use-gpu` option if you need to use your GPU from within the container (if you
need OpenGL for RViz, for example). If you want to run a command on container
startup, you can specify it by surrounding it with double quotes. Otherwise, a
shell (specifically, [zsh](https://en.wikipedia.org/wiki/Z_shell)) will launch.
For example:
  - `docker-run.sh`: Run the container and start the shell
  - `docker-run.sh --use-gpu`: Run the container with NVidia GPU support and
      start the shell 
  - `docker-run.sh "roscore"`: Run the container and start `roscore`
  - `docker-run.sh --use-gpu "roscore"`: Run the container with NVidia GPU
      support and start `roscore`

### Other things
You can also run `docker-join.sh` to attach a shell to the container. There
aren't any parameters for this. Just run it.

If you have to use an external camera (not including webcams) for some reason,
run `udev-aravis.sh` on your bare-metal computer (not in the container). You
only have to do this once.