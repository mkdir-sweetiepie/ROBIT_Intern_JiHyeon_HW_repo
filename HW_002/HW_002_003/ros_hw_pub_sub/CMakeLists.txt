## cmake 버전 설정
cmake_minimum_required(VERSION 2.8.3)


## 패키지 명
project(ros_hw_pub_sub)


## 의존성 패키지 추가
## 캐킨 빌드를 할 때 요구되는 구성요소 패키지이다.
## message_generation, std_msgs, roscpp이며 이 패키지들이 존재하지 않으면 빌드 도중에 에러가 난다.
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp)


## 메시지 선언
add_message_files(FILES Msg.msg)
## 의존하는 메시지 설정
## std_msgs가 설치되어 있지 않다면 빌드 도중에 에러가 난다.
generate_messages(DEPENDENCIES std_msgs)


## 캐킨 패키지 옵션으로 라이브러리, 캐킨 빌드 의존성, 시스템 의존 패키지를 기술한다.
catkin_package(
LIBRARIES ros_hw_pub_sub
CATKIN_DEPENDS std_msgs roscpp
)


## include 경로 설정
include_directories(${catkin_INCLUDE_DIRS})


## test_publisher 노드에 대한 빌드 옵션이다.
## 실행 파일, 타깃 링크 라이브러리, 추가 의존성 등을 설정한다.
add_executable(publisher src/publisher.cpp)
add_dependencies(publisher ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})
target_link_libraries(publisher ${catkin_LIBRARIES})


## test_subscriber 노드에 대한 빌드 옵션이다.
add_executable(subscriber src/subscriber.cpp)
add_dependencies(subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})
target_link_libraries(subscriber ${catkin_LIBRARIES})

