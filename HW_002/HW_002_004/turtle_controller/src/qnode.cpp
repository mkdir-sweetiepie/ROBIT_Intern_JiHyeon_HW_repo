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
#include "../include/turtle_controller/qnode.hpp"
#include <geometry_msgs/Twist.h>



/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace turtle_controller
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
    ros::init(init_argc, init_argv, "turtle_controller");
    if (!ros::master::check())
    {
      return false;
    }
    ros::start();  // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    msgpub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);


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

  void QNode::publeft()
  {
    geometry_msgs::Twist keydata;
    keydata.linear.x = keyx.linear.x;
    keydata.angular.z = keyz.angular.z;
    msgpub.publish(keydata);
    keydata.linear.x = keydata.angular.z = 0.0;
  }

  void QNode::pubright()
  {
    geometry_msgs::Twist keydata;
    keydata.linear.x = keyx.linear.x;
    keydata.angular.z = keyz.angular.z;
    msgpub.publish(keydata);
    keydata.linear.x = keydata.angular.z = 0.0;
  }

  void QNode::pubgo()
  {
    geometry_msgs::Twist keydata;
    keydata.linear.x = keyx.linear.x;
    keydata.angular.z = keyz.angular.z;
    msgpub.publish(keydata);
    keydata.linear.x = keydata.angular.z = 0.0;
  }

  void QNode::pubback()
  {
    geometry_msgs::Twist keydata;
    keydata.linear.x = keyx.linear.x;
    keydata.angular.z = keyz.angular.z;
    msgpub.publish(keydata);
    keydata.linear.x = keydata.angular.z = 0.0;
  }
  void QNode::pubkey()
  {
    geometry_msgs::Twist keydata;
    keydata.linear.x = keyx.linear.x;
    keydata.angular.z = keyz.angular.z;
    msgpub.publish(keydata);
    keydata.linear.x = keydata.angular.z = 0.0;
  }
  
}  // namespace s
