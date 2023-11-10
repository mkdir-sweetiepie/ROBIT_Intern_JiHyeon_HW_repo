/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/ros_qt_pub/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_qt_pub
{
  /*****************************************************************************
  ** Implementation
  *****************************************************************************/

  QNode::QNode(int argc, char** argv) : init_argc(argc), init_argv(argv)
  {
  }

  QNode::~QNode()
  {
    if (ros::isStarted())
    {
      ros::shutdown();  // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
  }

  bool QNode::init()
  {
    ros::init(init_argc, init_argv, "ros_qt_pub");
    if (!ros::master::check())
    {
      return false;
    }
    ros::start();  // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.

    // 퍼블리셔 선언 -> msgpub
  	// 토픽명 -> "pub_message"
  	// 퍼블리셔 큐 사이즈 -> 10
    msgpub = n.advertise<std_msgs::String>("/pub_message", 10);

    start();
    return true;
  }

  void QNode::run()
  {
    ros::Rate loop_rate(33);
    while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
  }

  //문자열 있을때 버튼 누르면 실행->메시지 발행
  void QNode::pubmsg()
  {
    if(!possible_pub)
    {
      std_msgs::String msg;
      msg.data = msgdata;
      msgpub.publish(msg);
      possible_pub = false; //초기화
    }
    else
      return;
  }

}  // namespace s
