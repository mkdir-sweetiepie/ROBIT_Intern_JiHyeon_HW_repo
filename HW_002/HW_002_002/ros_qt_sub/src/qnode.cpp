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
#include "../include/ros_qt_sub/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_qt_sub
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
    ros::init(init_argc, init_argv, "ros_qt_sub");
    if (!ros::master::check())
    {
      return false;
    }
    ros::start();  // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    // 서브스크라이버 선언 -> msgsub
    // 토픽명 -> "/ros_msg"
    // 서브스크라이버 큐 사이즈 -> 100
    msgsub = n.subscribe("/pub_message", 10, &QNode::submsgCallback, this);

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

  void QNode::submsgCallback(const std_msgs::String::ConstPtr& msg)
  {
    if (!possible_sub)
    {
      possible_sub = true;
      Q_EMIT msgsubSignal(msg->data);
    }
    else
      return;
  }

}  // namespace s
