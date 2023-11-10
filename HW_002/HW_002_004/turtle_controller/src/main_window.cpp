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
#include "../include/turtle_controller/main_window.hpp"
#include <unistd.h>
#include <termios.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace turtle_controller
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

  /*****************************************************************************
  ** Functions
  *****************************************************************************/

  void MainWindow::on_left_clicked()
  {
    qnode.keyx.linear.x = 0.0;
    qnode.keyz.angular.z = 2.0;
    qnode.publeft();
  }
  void MainWindow::on_right_clicked()
  {
    qnode.keyx.linear.x = 0.0;
    qnode.keyz.angular.z = -2.0;
    qnode.pubright();
  }
  void MainWindow::on_go_clicked()
  {
    qnode.keyx.linear.x = 2.0;
    qnode.keyz.angular.z = 0.0;
    qnode.pubgo();
  }
  void MainWindow::on_back_clicked()
  {
    qnode.keyx.linear.x= -2.0;
    qnode.keyz.angular.z= 0.0;
    qnode.pubback();
  }
  void MainWindow::keyPressEvent(QKeyEvent *event)
  {
    
    if(event->key() == 'W') 
    { 
      qnode.keyx.linear.x = 2.0; 
      qnode.keyz.angular.z = 0.0; 
      
    }
    else if(event->key() ==  'S') 
    {
      qnode.keyx.linear.x = -2.0; 
      qnode.keyz.angular.z =  0.0; 
       
    }
    else if(event->key() ==  'A') 
    {
      qnode.keyx.linear.x =  0.0; 
      qnode.keyz.angular.z =  2.0; 
      
    }
    else if(event->key() ==  'D') 
    { 
      qnode.keyx.linear.x =  0.0; 
      qnode.keyz.angular.z = -2.0; 
      
    }
    else if(event->key() ==  ' ') 
    { 
      qnode.keyx.linear.x =  0.0; 
      qnode.keyz.angular.z =  0.0; 

    }
    else;
    qnode.pubback();
  }

  
}  // namespace s
