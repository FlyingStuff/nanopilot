FROM ubuntu:18.04
MAINTAINER Patrick Spieler <stapelzeiger@gmail.com>

# steps for ROS2 Bouncy on Ubuntu Bionic
# https://github.com/ros2/ros2/wiki/Linux-Development-Setup

USER root


RUN apt update && apt install -y locales && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV TZ=America/Los_Angeles
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt update && apt install -y curl gnupg && rm -rf /var/lib/apt/lists/*
RUN curl http://repo.ros2.org/repos.key | apt-key add -
RUN sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu bionic main" > /etc/apt/sources.list.d/ros2-latest.list'

RUN apt update && apt install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-lark-parser \
    python3-pip \
    python-rosdep \
    python3-vcstool \
    wget \
 && rm -rf /var/lib/apt/lists/*

# install some pip packages needed for testing
RUN python3 -m pip install -U \
    argcomplete \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures

# [Ubuntu 16.04] install extra packages not available or recent enough on Xenial
RUN python3 -m pip install -U \
    pytest \
    pytest-cov \
    pytest-runner \
    setuptools

# install Fast-RTPS dependencies
RUN apt update &&  apt install --no-install-recommends -y \
    libasio-dev \
    libtinyxml2-dev \
 && rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -y \
    g++-arm-linux-gnueabihf \
    gcc-arm-linux-gnueabihf \
    g++-aarch64-linux-gnu \
    gcc-aarch64-linux-gnu \
 && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install -U numpy pytest