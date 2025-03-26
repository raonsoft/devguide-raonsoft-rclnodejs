# ROS2 개발 환경 (Docker)

이 프로젝트는 Ubuntu 22.04와 ROS2 Humble을 기반으로 한 Docker 개발 환경을 제공합니다.

## 요구사항

- Docker
- Docker Compose
- X11 서버 (macOS의 경우 XQuartz 필요)

## 설치 방법

1. Docker와 Docker Compose가 설치되어 있지 않다면 설치합니다.
2. macOS 사용자의 경우 XQuartz를 설치하고 실행합니다.
3. XQuartz 설정에서 "Allow connections from network clients" 옵션을 활성화합니다.
4. 터미널에서 다음 명령어를 실행하여 XQuartz를 재시작합니다:
   ```bash
   xhost + localhost
   ```

## 개발 환경 실행

1. Docker 이미지 빌드 및 컨테이너 실행:
   ```bash
   docker-compose up -d
   ```

2. 컨테이너에 접속:
   ```bash
   docker-compose exec ros2_dev bash
   ```

3. ROS2 워크스페이스 초기화:
   ```bash
   cd /workspace
   ./init_workspace.sh
   ```

4. ROS2 환경이 자동으로 설정됩니다. 다음 명령어로 확인할 수 있습니다:
   ```bash
   ros2 --version
   ```

## ROS2 패키지 개발

1. 새로운 패키지 생성:
   ```bash
   cd /workspace/ros2_ws/src
   ros2 pkg create --build-type ament_python my_package
   ```

2. 패키지 빌드:
   ```bash
   cd /workspace/ros2_ws
   colcon build --symlink-install
   ```

3. 환경 설정:
   ```bash
   source /workspace/ros2_ws/install/setup.bash
   ```

## 사용 가능한 개발 도구

- ROS2 Humble
- Python 3
- Node.js 18.x
- rclnodejs

## 작업 디렉토리 구조

- `/workspace/ros2_ws`: ROS2 워크스페이스
  - `src/`: 패키지 소스 코드
  - `build/`: 빌드 파일
  - `install/`: 설치된 파일
  - `log/`: 빌드 로그

## 주의사항

- GUI 애플리케이션을 실행하려면 XQuartz가 실행 중이어야 합니다.
- 컨테이너를 종료하려면 `docker-compose down` 명령어를 사용합니다.
- 패키지를 수정한 후에는 반드시 `colcon build --symlink-install` 명령어로 다시 빌드해야 합니다. 