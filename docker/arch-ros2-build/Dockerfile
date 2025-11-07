FROM cachyos/cachyos-v4:latest

ARG USER_ID
ARG GROUP_ID

RUN rm -f /var/cache/pacman/pkg/*


RUN rm -rf /etc/pacman.d/gnupg/
RUN pacman-key --init
RUN pacman-key --populate

RUN pacman -Sy
RUN pacman -S --noconfirm cmake

