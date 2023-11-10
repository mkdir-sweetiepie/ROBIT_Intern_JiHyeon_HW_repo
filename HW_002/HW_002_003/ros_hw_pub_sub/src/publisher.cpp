#include "ros/ros.h" // ROS 기본 헤더파일
#include "ros_hw_pub_sub/Msg.h"// Msg 메시지 파일 헤더(빌드 후 자동 생성됨)
#include "iostream"




int main(int argc, char **argv) // 노드 메인 함수
{
	ros::init(argc, argv, "publisher"); // 노드명 초기화
	ros::NodeHandle nh; // ROS 시스템과 통신을 위한 노드 핸들 선언

	// 퍼블리셔 선언, ros_hw_pub_sub 패키지의 Msg 메시지 파일을 이용한
	// 퍼블리셔 ros_pub 를 작성한다. 토픽명은 "ros_msg" 이며,
	// 퍼블리셔 큐(queue) 사이즈를 100개로 설정한다는 것이다
	// 퍼블리셔 선언 -> ros_pub
  	// 토픽명 -> "ros_msg"
  	// 퍼블리셔 큐 사이즈 -> 100
	ros::Publisher ros_pub = nh.advertise<ros_hw_pub_sub::Msg>("ros_msg", 100);

	// 루프 주기를 설정한다. "10" 이라는 것은 10Hz,0.1초 간격으로 반복된다
	ros::Rate loop_rate(10);

	// Msg 메시지 파일 형식으로 msg 라는 메시지를 선언
	ros_hw_pub_sub::Msg msg;

	// 메시지에 사용될 변수 선언
	int data1;
	int data2;
	std::string message;
	// ros가 실행중일때
	while (ros::ok())
	{
		// 사용자로부터 입력 받음
		nh.getParam("data1", data1);;
		nh.getParam("data2", data2);
		nh.getParam("message", message);

        // 입력 받은 값으로 메시지 설정
        msg.stamp = ros::Time::now();
        msg.data1 = data1;
        msg.data2 = data2;
        msg.message = message;

		ROS_INFO("publish msg time(stmap)= %d %d", msg.stamp.sec , msg.stamp.nsec); // stamp.sec 메시지를 표시한다
		ROS_INFO("publish msg data1(int64)s= %ld", msg.data1); // data1 메시지를 표시한다
		ROS_INFO("publish msg data2(int64)= %ld", msg.data2); // data2 메시지를 표시한다
		ROS_INFO("publish msg message(string)= %s", msg.message.c_str()); // message 메시지를 표시한다
		ROS_INFO("                      ");


		ros_pub.publish(msg); // 메시지를 발행한다
		loop_rate.sleep(); // 위에서 정한 루프 주기에 따라 슬립에 들어간다
		
	}
	return 0;
}
