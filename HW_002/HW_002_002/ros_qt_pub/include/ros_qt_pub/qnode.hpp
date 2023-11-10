/**
 * @file /include/ros_qt_pub/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ros_qt_pub_QNODE_HPP_
#define ros_qt_pub_QNODE_HPP_

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


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_qt_pub
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

    //pub 가능한지 확인하는 변수 처음애는 false
    bool possible_pub = true;
    //문자열 데이터 변수
    std::string msgdata;
    //메시지 pub하는 함수
    void pubmsg();


  Q_SIGNALS:
    void rosShutdown();

  private:
    int init_argc;
    char** init_argv;

    //퍼블리셔 선언 -> msgpub
    ros::Publisher msgpub;
  };

}  // namespace s

#endif /* ros_qt_pub_QNODE_HPP_ */
