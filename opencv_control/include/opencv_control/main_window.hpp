/**
 * @file /include/opencv_control/main_window.hpp
 *
 * @brief Qt based gui for opencv_control.
 *
 * @date November 2010
 **/
#ifndef opencv_control_MAIN_WINDOW_H
#define opencv_control_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>

#ifndef Q_MOC_RUN
#include "ui_main_window.h"
#include "qnode.hpp"
#endif

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace opencv_control {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    //void updateLoggingView(); // no idea why this can't connect automatically
  void on_bottom1_sliderMoved(int position);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace opencv_control

#endif // opencv_control_MAIN_WINDOW_H
