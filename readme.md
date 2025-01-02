# ROS Topic Browser PRD
![ROS Topic Browser Screenshot](./src/ros_web_monitor/document/web.png)

## 1. 개요
ROS 토픽을 웹 브라우저에서 모니터링할 수 있는 웹 기반 도구입니다.

## 2. 주요 기능
- ROS 토픽 실시간 모니터링
- 토픽 메시지 내용 표시 (JSON/YAML 형식)
- 토픽 타입 표시
- 토픽 발행 주기(Hz) 실시간 계산
- 시스템 토픽 필터링 (토글 가능)
- 토픽 검색 기능
- 즐겨찾기 기능 (로컬 스토리지 저장)
- 다크/라이트 모드 지원 (사용자 설정 저장)

## 3. 시스템 요구사항
### 필수 패키지
- ROS Noetic
- rosbridge_server
- Python 3.x
- 웹 브라우저 (Chrome, Firefox, Safari, Edge 지원)

### 설치 방법
```bash
# ROS Bridge 설치
sudo apt-get install ros-noetic-rosbridge-server

# 워크스페이스 생성
mkdir -p ~/topic_browser/src
cd ~/topic_browser/src
catkin_init_workspace

# 패키지 복제
git clone [repository_url] ros_web_monitor
```

## 4. 실행 방법
### 방법 1: 개별 실행
```bash
# Terminal 1
roscore

# Terminal 2
roslaunch rosbridge_server rosbridge_websocket.launch

# Terminal 3
cd src/ros_web_monitor/web
python3 -m http.server 8081
```

### 방법 2: 스크립트 실행
```bash
# 스크립트 실행 권한 설정
chmod +x src/ros_web_monitor/scripts/start_monitor.sh

# 스크립트 실행
src/ros_web_monitor/scripts/start_monitor.sh
```

## 5. 사용자 인터페이스
- ROS 연결 상태 표시
- 북마크된 토픽 섹션
- 전체 토픽 목록 섹션
- 다크/라이트 모드 토글
- 시스템 토픽 숨기기 토글
- 토픽 검색창

## 6. 디렉토리 구조
```
topic_browser/
├── src/
│   └── ros_web_monitor/
│       ├── web/
│       │   ├── index.html
│       │   ├── css/
│       │   │   └── styles.css
│       │   └── js/
│       │       └── config.js
│       ├── scripts/
│       │   └── start_monitor.sh
└── readme.md
```

## 7. 기술 스택
- Frontend: HTML, CSS, JavaScript
- ROS Bridge: rosbridge_websocket
- WebSocket: roslibjs
- Web Server: Python http.server
- 로컬 스토리지: 사용자 설정 저장

## 8. 브라우저 지원
- Chrome (권장)
- Firefox
- Safari
- Edge

## 9. 알려진 제한사항
- 웹소켓 연결이 불안정할 경우 재연결 필요
- 대용량 메시지의 경우 표시 지연 발생 가능
- 토픽 발행 주기가 매우 빠른 경우 UI 업데이트 제한
- 시스템 토픽 목록이 하드코딩되어 있음