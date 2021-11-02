FROM ubuntu:20.04

ENV TZ=Europe/Germany
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt update
RUN apt install -y tzdata
RUN apt install -y gcc-arm-none-eabi
RUN apt install -y git make cmake
