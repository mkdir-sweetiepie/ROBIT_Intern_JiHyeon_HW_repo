/**
 * @file /include/udp_send/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date November 2010
 **/
#ifndef udp_send_MAIN_WINDOW_H
#define udp_send_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "udp.h"
#include "QHostAddress"
#include "QUdpSocket"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace udp_send
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

    //IP 주소 및 포트 설정
    QHostAddress my_IP;
    QHostAddress your_IP;
    

    char q[3];
    
  public Q_SLOTS:
    void on_left_clicked();
    void on_right_clicked();
    void on_go_clicked();
    void on_back_clicked();

    void keyPressEvent(QKeyEvent *event);
    void send();
    
  private:
    Ui::MainWindowDesign ui;
    uint16_t PORT;
    QUdpSocket* send_socket;
    QNode qnode;
  };

}  // namespace s

#endif  // udp_send_MAIN_WINDOW_H
