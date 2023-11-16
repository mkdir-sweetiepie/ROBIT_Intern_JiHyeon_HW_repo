/**
 * @file /include/listener/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date November 2010
 **/
#ifndef listener_MAIN_WINDOW_H
#define listener_MAIN_WINDOW_H

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

namespace listener
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

    QHostAddress IP;
    QUdpSocket* ocam1_UdpSocket;
    uint16_t PORT;
    cv::Mat ocam1_image;

  public Q_SLOTS:
    void ocam1_read();
    void Decoding_Datagram_ocam1(QByteArray inputDatagram);
    cv::Mat canny_edge(cv::Mat image);
    cv::Mat region_of_interest(cv::Mat image);
    void bi(void);
    void analyzeConesPosition();
    cv::Point findWhiteLinePosition(const cv::Mat& image);
 
  private:
    Ui::MainWindowDesign ui;
    QNode qnode;
  };

}  // namespace s

#endif  // listener_MAIN_WINDOW_H
