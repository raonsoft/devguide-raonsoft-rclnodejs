#!/bin/bash

# ROS2 워크스페이스 생성
mkdir -p ros2_ws/src
cd ros2_ws

# 워크스페이스 초기화
colcon build --symlink-install

# 워크스페이스 설정을 .bashrc에 추가
echo "source /workspace/ros2_ws/install/setup.bash" >> /root/.bashrc

# 워크스페이스 권한 설정
chmod -R 777 /workspace/ros2_ws 