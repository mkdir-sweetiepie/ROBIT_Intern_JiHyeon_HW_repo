/**
 * @file /include/turtle_controller/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef turtle_controller_QNODE_HPP_
#define turtle_controller_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <geometry_msgs/Twist.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace turtle_controller
{
  /*****************************************************************************
  ** Class
  *****************************************************************************/

  class QNode : public QThread
  {
    Q_OBJECT
  public:
    QNode(int argc, char** argv);
    virtual ~QNode();
    bool init();
    void run();

    geometry_msgs::Twist keyx;
    geometry_msgs::Twist keyz;


    void publeft();
    void pubright();
    void pubgo();
    void pubback();
    void pubkey();


  Q_SIGNALS:
    void rosShutdown();

  private:
    int init_argc;
    char** init_argv;

    ros::Publisher msgpub;
  };

}  // namespace s

#endif /* turtle_controller_QNODE_HPP_ */
