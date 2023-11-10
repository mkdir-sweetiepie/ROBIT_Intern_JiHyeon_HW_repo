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
#include "../include/ros_qt_pub/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_qt_pub
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
  }

  MainWindow::~MainWindow()
  {
  }

  void MainWindow::on_pushButton_clicked()
  {
    //lineEdit에 입력된 글자가 없으면
    if(ui.lineEdit->text().isEmpty())
    {
      QMessageBox::warning(this, "QMessageBox", "Warning: message X");
      qnode.possible_pub = true;
      return;
    }
    //있으면 
    else
    {

      qnode.possible_pub = false;
      qnode.msgdata = ui.lineEdit->text().toStdString();
      qnode.pubmsg();
      ui.lineEdit->clear();
    }
  }
  
  /*****************************************************************************
  ** Functions
  *****************************************************************************/

}  // namespace s
