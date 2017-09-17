#ifndef SERIAL_H
#define SERIAL_H

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <iostream>
#include "ros/ros.h"
//#include "serial_node/data.h"


typedef unsigned char uchar;

class Serial
{
public:
	Serial(const char* _serial_port, unsigned long _baud_rate);
	int begin();
	float bytesToFloat(uchar b0, uchar b1, uchar b2, uchar b3);
	void flush();

private:
	int fd;
	unsigned long baud_rate;
	const char* serial_port;
    struct termios toptions;

};

#endif
