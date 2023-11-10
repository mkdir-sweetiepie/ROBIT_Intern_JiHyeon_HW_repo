/**
 * @file /include/ros_qt_sub/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date November 2010
 **/
#ifndef ros_qt_sub_MAIN_WINDOW_H
#define ros_qt_sub_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QMetaType>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace ros_qt_sub
{
  /*****************************************************************************
  ** Interface [MainWindow]
  *****************************************************************************/
  /**
   * @brief Qt central, all operations relating to the view part here.
   */
  class MainWindow : public QMainWindow
  {
    Q_OBJECT

  public:
    MainWindow(int argc, char** argv, QWidget* parent = 0);
    ~MainWindow();

  public Q_SLOTS:
    void msgsubCallback(std::string msg);

  private:
    Ui::MainWindowDesign ui;
    QNode qnode;
  };

}  // namespace s

#endif  // ros_qt_sub_MAIN_WINDOW_H
