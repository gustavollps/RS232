/**
 * @file /include/opencv_control/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef opencv_control_QNODE_HPP_
#define opencv_control_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#endif


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace opencv_control {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

  uint8_t b_1;
  uint8_t b_2;
  uint8_t b_3;
  uint8_t t_1;
  uint8_t t_2;
  uint8_t t_3;

  //QStringListModel* loggingModel() { return &logging_model; }
  //void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
  //void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
  ros::Publisher color_pub_;
  //  QStringListModel logging_model;
};

}  // namespace opencv_control

#endif /* opencv_control_QNODE_HPP_ */
