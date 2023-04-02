FROM ubuntu:22.04

ENV TZ=Europe/Germany
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN useradd -u 1000 -m nxt2

RUN apt update
RUN apt install -y tzdata
RUN apt install -y build-essential gcc g++
RUN apt install -y make cmake git 
RUN apt install -y gcc-arm-none-eabi

USER nxt2

