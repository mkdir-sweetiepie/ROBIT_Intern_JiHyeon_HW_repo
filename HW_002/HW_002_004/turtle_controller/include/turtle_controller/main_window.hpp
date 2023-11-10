/**
 * @file /include/turtle_controller/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date November 2010
 **/
#ifndef turtle_controller_MAIN_WINDOW_H
#define turtle_controller_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace turtle_controller
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
    void on_left_clicked();
    void on_right_clicked();
    void on_go_clicked();
    void on_back_clicked();

    void keyPressEvent(QKeyEvent *event);
    
   
    
  private:
    Ui::MainWindowDesign ui;
    QNode qnode;
  };

}  // namespace s

#endif  // turtle_controller_MAIN_WINDOW_H
