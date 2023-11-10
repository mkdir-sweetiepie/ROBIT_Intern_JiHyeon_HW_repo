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
#include "../include/udp_send/main_window.hpp"
#include "QByteArray"
#include "QHostAddress"
#include "QUdpSocket"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace udp_send
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

    my_IP=QHostAddress("192.168.0.73");
    your_IP= QHostAddress("192.168.0.63");
    PORT = 5000;

    send_socket = new QUdpSocket;

    
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
    // qnode.x = 0;
    // qnode.z = 2;
    // qnode.send();
  }
  void MainWindow::on_right_clicked()
  {
    // qnode.x = 0;
    // qnode.z = -2;
    // qnode.send();
  }
  void MainWindow::on_go_clicked()
  {
    // qnode.x = 2;
    // qnode.z = 0;
    // qnode.send();
  }
  void MainWindow::on_back_clicked()
  {
    // qnode.x= -2;
    // qnode.z= 0;
    // qnode.send();
  }
  void MainWindow::keyPressEvent(QKeyEvent *event)
  {
    
    if(event->key() == 'W') 
    { 
      q[0]='W';
      q[1]='2';
      send();
    }
    else if(event->key() ==  'S') 
    {
      q[0]='S';
      q[1]='2';
      send();
       
    }
    else if(event->key() ==  'A') 
    {
      q[0]='A';
      q[1]='2';
      send();

    }
    else if(event->key() ==  'D') 
    { 
      q[0]='D';
      q[1]='2';
      send();
      
    }
    
    else;
    
  }
  void MainWindow::send()
  {
    
    QByteArray array;

    //입력 데이터를 QByteArray에 추가
    array.append(q);
    int sendok;
    //데이터를 지정된 IP와 포트로 송신
    sendok = send_socket->writeDatagram(array.data(), array.size(),your_IP, PORT);
    
  }
} 