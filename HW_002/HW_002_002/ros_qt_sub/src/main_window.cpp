/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/ros_qt_sub/main_window.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_qt_sub
{
  using namespace Qt;

  /*****************************************************************************
  ** Implementation [MainWindow]
  *****************************************************************************/

  MainWindow::MainWindow(int argc, char** argv, QWidget* parent) : QMainWindow(parent), qnode(argc, argv)
  {
    ui.setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    setWindowIcon(QIcon(":/images/icon.png"));

    qnode.init();

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    qRegisterMetaType<std::string>("std::string");
    QObject::connect(&qnode, SIGNAL(msgsubSignal(std::string)), this, SLOT(msgsubCallback(std::string)));
  }

  MainWindow::~MainWindow()
  {
  }

  /*****************************************************************************
  ** Functions
  *****************************************************************************/

  void MainWindow::msgsubCallback(std::string msg)
  {
    ui.lineEdit->clear();
    QString submsg = QString::fromStdString(msg);
    ui.lineEdit->setText(submsg);
    qnode.possible_sub = false;
  }

}  // namespace s
