/**
 * @file /include/image_viewer_udp/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date November 2010
 **/
#ifndef image_viewer_udp_MAIN_WINDOW_H
#define image_viewer_udp_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "udp.h"
#include <opencv2/opencv.hpp>
#include <QImage>
#include <QtNetwork/QUdpSocket>
#include <QHostAddress>


/*****************************************************************************
** Namespace
*****************************************************************************/

namespace image_viewer_udp
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

    // //resized_image 해상도
    // int imgResizeWidth = 320;
    // int imgResizeHeight = 180;

    // QHostAddress IP;
    // QUdpSocket* ocam1_UdpSocket;
    // uint16_t PORT;
    // cv::Mat ocam1_image;
  public Q_SLOTS:
    void slotUpdateImg();
    // void ocam1_read();
    // void Decoding_Datagram_ocam1(QByteArray inputDatagram);
  
  private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    
  };

}  // namespace s

#endif  // image_viewer_udp_MAIN_WINDOW_H
