FROM nvidia/cuda:12.1.0-cudnn8-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul

# NVIDIA GPU 렌더링 지원
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=utility,compute

# ───────────────────────────────────────────
# 로케일 설정 (ROS2 필수)
# ───────────────────────────────────────────
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ───────────────────────────────────────────
# 시스템 패키지
# ───────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    wget curl ca-certificates gnupg \
    git unzip zip \
    software-properties-common \
    iproute2 iputils-ping net-tools \
    gedit tree \
    # Python 3.10 (SB3 및 ROS2 Humble 공통)
    python3 python3-pip python3-venv python3-dev \
    && rm -rf /var/lib/apt/lists/*

# ───────────────────────────────────────────
# ROS2 Humble 설치
# Ubuntu 22.04(Jammy) 공식 지원, CycloneDDS 기본 RMW
# ───────────────────────────────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu jammy main" \
        > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-rmw-cyclonedds-cpp \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# rosdep 초기화
RUN rosdep init && rosdep update

# ROS2 환경변수
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# ───────────────────────────────────────────
# RL 가상환경 (시스템 패키지 충돌 방지)
# PyTorch >= 2.3 + CUDA 12.1 + SB3
# ───────────────────────────────────────────
RUN python3 -m venv /opt/venv/rl

COPY stable-baselines3/ /tmp/stable-baselines3/

RUN /opt/venv/rl/bin/pip install --upgrade pip && \
    /opt/venv/rl/bin/pip install "torch>=2.3,<3.0" \
        --index-url https://download.pytorch.org/whl/cu121 && \
    /opt/venv/rl/bin/pip install -e '/tmp/stable-baselines3[extra]' && \
    /opt/venv/rl/bin/pip install \
        "python-socketio[client]" \
        eventlet \
        gevent \
        gevent-websocket \
        pyyaml

# 컨테이너 접속 시 RL 가상환경 자동 활성화
RUN echo "source /opt/venv/rl/bin/activate" >> /root/.bashrc

# ───────────────────────────────────────────
# CycloneDDS 설정 파일 경로 (볼륨 마운트 후 유효)
# ───────────────────────────────────────────
ENV CYCLONEDDS_URI=/autodrive/cyclonedds_config.xml

# ───────────────────────────────────────────
# 작업 디렉토리 (호스트와 공유되는 폴더)
# ───────────────────────────────────────────
WORKDIR /autodrive

CMD ["/bin/bash"]
