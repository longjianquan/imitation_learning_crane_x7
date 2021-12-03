FROM nvidia/cuda:11.3.0-devel-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive

# Force color prompt in terminal.
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /root/.bashrc

# japanise setting
RUN apt-get update
RUN apt-get -y install locales && \
    localedef -f UTF-8 -i ja_JP ja_JP.UTF-8
ENV LANG ja_JP.UTF-8
ENV LANGUAGE ja_JP:ja
ENV LC_ALL ja_JP.UTF-8
ENV TZ JST-9
ENV TERM xterm

# install utility
RUN apt-get update && \
    apt-get install -y \
    wget \
    curl \
    git \
    vim \
    python3-pip \
    ffmpeg

# install python package
WORKDIR /root/requirements
ADD requirements.txt /root/requirements
RUN pip3 install -r requirements.txt
RUN pip3 install torch==1.9.1+cu111 torchvision==0.10.1+cu111 torchaudio==0.9.1 -f https://download.pytorch.org/whl/torch_stable.html

# fix error
RUN apt update && apt install -y libsdl2-dev

# install fish
RUN apt-get update && apt-get install -y fish
SHELL ["/usr/bin/fish", "-c"]

# install fish plagin
RUN curl https://git.io/fisher --create-dirs -sLo ~/.config/fish/functions/fisher.fish
RUN fisher install oh-my-fish/theme-bobthefish
RUN fisher install jethrokuan/z

RUN export DISPLAY=:0

WORKDIR /root/workspace
