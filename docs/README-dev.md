# Development Instructions

## Docker setup

```shell
sudo apt update
sudo apt -y install docker.io
sudo usermod --groups docker --append $(id -u -n)
```

Now log out and in again to ensure you're a member of the `docker` group.

## Building

These instructions should work on any system that has a recent verion of
Docker installed. For example, native Ubuntu Linux 22.04, or Ubuntu Linux
under WSL (Windows Subsystem for Linux).

Espressif publishes Docker images containing IDF, so you no longer need to
install IDF locally. `build.sh` will download the relevant Docker image, run
`idf.py` as required, etc.

Note that `build.sh` bind-mounts your home directory and the source code from
your host into the (ephemeral) Docker container. Files will be saved in
`~/.cache/Espressif`, `~/.ccache` and perhaps other directories.

### Build

```shell
./build.sh
```

### Clean

```shell
./build.sh idf.py fullclean
```

### Enter Docker Container for Experimentation

```shell
./build.sh bash
```

## Flashing

Replace `/dev/ttyUSB0` with the relevant port on your host:

```shell
sudo chmod 666 /dev/ttyUSB0
./build.sh idf.py -p /dev/ttyUSB0 flash
```
