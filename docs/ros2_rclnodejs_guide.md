# ROS2 rclnodejs 개발 환경 설정 가이드

## 목차
1. [개요](#개요)
2. [환경 요구사항](#환경-요구사항)
3. [설치 과정](#설치-과정)
4. [프로젝트 구조](#프로젝트-구조)
5. [실행 방법](#실행-방법)
6. [주의사항](#주의사항)

## 개요
이 가이드는 ROS2 Humble과 Node.js를 사용하여 TypeScript로 ROS2 노드를 개발하는 방법을 설명합니다. Docker 기반의 개발 환경을 사용하여 설정의 일관성을 유지합니다.

## 환경 요구사항
- Docker
- Docker Compose
- X11 서버 (macOS의 경우 XQuartz 필요)
- Node.js 23.x
- TypeScript
- ROS2 Humble

## 설치 과정

### 1. ROS2 개발 환경 설정
```bash
# Docker 컨테이너 실행
docker-compose up -d

# 컨테이너 접속
docker-compose exec ros2_dev bash
```

### 2. ROS2 워크스페이스 초기화
```bash
# 워크스페이스 디렉토리 생성 및 초기화
cd /workspace
./init_workspace.sh

# 워크스페이스 빌드
cd /workspace/ros2_ws
colcon build --symlink-install

# 환경 설정
source /workspace/ros2_ws/install/setup.bash
```

### 3. rclnodejs 프로젝트 생성
```bash
# 프로젝트 디렉토리 생성 및 npm 초기화
cd /workspace/ros2_ws/src
mkdir -p node-ros2-demo
cd node-ros2-demo
npm init -y

# rclnodejs 설치 및 초기화
npm install rclnodejs
npx rclnodejs init

# TypeScript 관련 패키지 설치
npm install typescript @types/node ts-node --save-dev
```

### 4. TypeScript 설정
```bash
# tsconfig.json 생성
cat > tsconfig.json << EOF
{
  "compilerOptions": {
    "target": "es6",
    "module": "commonjs",
    "moduleResolution": "node",
    "outDir": "./dist",
    "rootDir": "./src",
    "strict": true,
    "esModuleInterop": true,
    "skipLibCheck": true,
    "forceConsistentCasingInFileNames": true,
    "resolveJsonModule": true
  },
  "include": ["src/**/*"],
  "exclude": ["node_modules"]
}
EOF
```

### 5. package.json 스크립트 설정
```bash
# package.json에 스크립트 추가
npm pkg set scripts.build="tsc"
npm pkg set scripts.start="ts-node src/listener.ts"
```

### 6. 소스 코드 작성
```bash
# src 디렉토리 생성
mkdir -p src

# listener.ts 파일 생성
cat > src/listener.ts << EOF
import * as rclnodejs from 'rclnodejs';

rclnodejs.init().then(() => {
  const node = new rclnodejs.Node('js_listener');
  const subscription = node.createSubscription(
    'std_msgs/msg/String',
    'chatter',
    (msg: rclnodejs.std_msgs.msg.String) => {
      console.log('Received: ' + msg.data);
    }
  );
  rclnodejs.spin(node);
}).catch((err: Error) => {
  console.error('Error:', err);
});
EOF
```

### 7. 프로젝트 빌드 및 실행
```bash
# 워크스페이스 빌드
cd /workspace/ros2_ws
colcon build --symlink-install

# 환경 설정
source /workspace/ros2_ws/install/setup.bash

# listener 실행
cd /workspace/ros2_ws/src/node-ros2-demo
npm run start
```

### 8. 테스트를 위한 talker 실행
```bash
# 다른 터미널에서
docker-compose exec ros2_dev bash -c "source /opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp talker"
```

## 프로젝트 구조
```
ros2_ws/
├── src/
│   └── node-ros2-demo/
│       ├── src/
│       │   └── listener.ts
│       ├── package.json
│       ├── tsconfig.json
│       └── node_modules/
├── build/
├── install/
└── log/
```

## 실행 방법
1. ROS2 환경 설정:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. 워크스페이스 환경 설정:
   ```bash
   source /workspace/ros2_ws/install/setup.bash
   ```

3. TypeScript 코드 컴파일:
   ```bash
   npm run build
   ```

4. 노드 실행:
   ```bash
   npm run start
   ```

## 주의사항
1. ROS2 환경이 항상 로드되어 있어야 함 (`source /opt/ros/humble/setup.bash`)
2. 워크스페이스 환경도 로드되어 있어야 함 (`source /workspace/ros2_ws/install/setup.bash`)
3. TypeScript 코드를 수정한 후에는 `npm run build`로 컴파일 필요
4. Docker 컨테이너를 재시작할 경우 환경 설정을 다시 해야 함
5. macOS 사용자의 경우 XQuartz 설정이 필요함
6. GUI 애플리케이션 실행을 위해 XQuartz가 실행 중이어야 함 