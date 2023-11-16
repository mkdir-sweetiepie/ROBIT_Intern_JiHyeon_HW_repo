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
#include "../include/image_viewer_udp/main_window.hpp"
#include "QByteArray"
#include <opencv2/opencv.hpp>
#include <QImage>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace image_viewer_udp
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

  //수신 할때 실행할 함수 설정
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  // subscribe한 이미지의 callback에서 시그널을 발생시켜 위 함수를 부른다.
  QObject::connect(&qnode, SIGNAL(sigRcvImg()), this, SLOT(slotUpdateImg()));
}

MainWindow::~MainWindow()
{
}

/*****************************************************************************
** Functions
*****************************************************************************/

//이미지 처리 및 UI 업데이트를 수행하며 이후 UDP를 통한 이미지 전송을 위한 시그널을 발생
void MainWindow::slotUpdateImg()
{
  // show callback image
  // QImage 객체인 raw_image를 생성//QImage는 Qt 프레임워크에서 이미지를 다루기 위한 클래스
  // QImage는 Qt 프레임워크에서 이미지를 다루기 위한 클래스//qnode.Original_ocam1.cols와 qnode.Original_ocam1.rows는
  // 이미지의 너비와 높이 QImage::Format_RGB888는 이미지의 형식
  QImage qImage((const unsigned char*)(qnode.imgView.data), qnode.imgView.cols, qnode.imgView.rows,
                QImage::Format_RGB888);
  ui.label->setPixmap(QPixmap::fromImage(qImage.rgbSwapped()));
}

}  // namespace image_viewer_udp
