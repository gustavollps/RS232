#include "../../include/pwm/Serial.h"

Serial::Serial(const char* _serial_port, unsigned long _baud_rate){
	serial_port = _serial_port;
	switch(_baud_rate){
		case 115200:
			baud_rate = B115200;
			break;
		case 57600:
			baud_rate = B57600;
			break;
		case 38400:
			baud_rate = B38400;
			break;
		case 19200:
			baud_rate = B19200;
			break;
		case 9600:
			baud_rate = B9600;
			break;
		case 4800:
			baud_rate = B4800;
			break;
		case 2400:
			baud_rate = B2400;
			break;
		default:
			baud_rate = B115200;
			break;
	}
}

float Serial::bytesToFloat(uchar b0, uchar b1, uchar b2, uchar b3)
{
    float output;

    *((uchar*)(&output) + 3) = b3;
    *((uchar*)(&output) + 2) = b2;
    *((uchar*)(&output) + 1) = b1;
    *((uchar*)(&output) + 0) = b0;

    return output;
}

void Serial::flush(){
	char flush[100];
	int n = read (fd, flush, sizeof flush);
}

int Serial::begin(){

	/* open serial port */
	fd = open(serial_port, O_RDWR | O_NOCTTY);
  	printf("fd opened as %i\n", fd);
  
  	/* wait for the Arduino to reboot */
  	sleep(3);
  
  	/* get current serial port settings */
  	tcgetattr(fd, &toptions);
  	/* set 115200 baud both ways */
  	cfsetispeed(&toptions, baud_rate);
  	cfsetospeed(&toptions, baud_rate);
  
  	/* 8 bits, no parity, no stop bits */
  	toptions.c_cflag &= ~PARENB;
  	toptions.c_cflag &= ~CSTOPB;
  	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;

  	/* No Canonical mode */
  	toptions.c_lflag = 0;

  	/* commit the serial port settings */
  	tcsetattr(fd, TCSANOW, &toptions);
  	return fd;
}
