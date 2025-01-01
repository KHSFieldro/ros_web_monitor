#!/bin/bash

# ROS 환경 설정
# source /opt/ros/noetic/setup.bash
# source ~/Dev/topic_browser/devel/setup.bash

# 현재 작업 디렉토리 설정
cd ~/Dev/topic_browser/src/ros_web_monitor/web

# rosbridge 서버 실행 (백그라운드)
roslaunch rosbridge_server rosbridge_websocket.launch &
ROSBRIDGE_PID=$!

# 잠시 대기 (rosbridge가 완전히 시작될 때까지)
sleep 2

# 웹 서버 실행 (백그라운드)
python3 -m http.server 8081 &
HTTP_PID=$!

# 프로세스 종료 처리 함수
cleanup() {
    echo "서버 종료 중..."
    kill $HTTP_PID
    kill $ROSBRIDGE_PID
    exit 0
}

# Ctrl+C 시그널 처리
trap cleanup SIGINT

echo "ROS Topic Browser 서버가 시작되었습니다."
echo "웹 브라우저에서 http://localhost:8081 접속"
echo "종료하려면 Ctrl+C를 누르세요."

# 스크립트 실행 유지
wait