FROM ubuntu:22.04

# 환경 변수 설정
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# 기본 패키지 설치
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# ROS2 저장소 추가 및 설치
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# ROS2 환경 설정
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

# Node.js 설치 (버전 23.x)
RUN curl -fsSL https://deb.nodesource.com/setup_23.x | bash -
RUN apt-get install -y nodejs

# rclnodejs 설치를 위한 의존성 설치
RUN apt-get update && apt-get install -y \
    python3-dev \
    build-essential \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install setuptools

# npm 전역 패키지 설치
RUN npm install -g npm@latest
RUN npm install -g node-gyp
RUN npm install -g rimraf

# ROS2 환경을 로드하고 rclnodejs 설치
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && npm install -g rclnodejs"

# 작업 디렉토리 설정
WORKDIR /workspace

# 워크스페이스 초기화 스크립트 복사 및 실행 권한 설정
COPY init_workspace.sh /workspace/
RUN chmod +x /workspace/init_workspace.sh

# 컨테이너 실행 시 실행할 명령어
CMD ["/bin/bash"] 