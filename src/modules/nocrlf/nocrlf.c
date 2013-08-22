/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
 *           Anton Babushkin <anton.babushkin@me.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file nocrlf.c
 *
 * Simple program to turn off the conversion of LF to CRLF (\n to \r\n) on the serial port /dev/ttyACM0.
 *
 * @author Peter Adamczyk <p.g.adamczyk@gmail.com>
 */

//#include <nuttx/config.h>
#include <unistd.h>
//#include <pthread.h>
#include <stdio.h>
//#include <math.h>
//#include <stdbool.h>
#include <fcntl.h>
//#include <mqueue.h>
//#include <string.h>
//#include <drivers/drv_hrt.h>
//#include <time.h>
//#include <float.h>
//#include <unistd.h>
//#include <nuttx/sched.h>
//#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <stdlib.h>
//#include <poll.h>

//#include <systemlib/param/param.h>
//#include <systemlib/systemlib.h>
#include <systemlib/err.h>
//#include <mavlink/mavlink_log.h>

__EXPORT int nocrlf_main(void) ;

int nocrlf_main(void)
{

	// Hard-coded port name: usb serial terminal port.
char *uart_name = "/dev/ttyACM0";

int uart = open(uart_name, O_RDWR | O_NOCTTY);
warnx("UART is %s, status is %d\n", uart_name, uart);

/* Get parameters */
struct termios uart_config;
int termios_state;

/* Fill with the original uart configuration. */
if ((termios_state = tcgetattr(uart, &uart_config)) < 0) {
warnx("ERROR get termios config %s: %d\n", uart_name, termios_state);
close(uart);
return -1;
}

///* Try to Get and Set baud rate  - COMMENTED because it does not seem to affect the USB Port */
//int ispeed;
//int ospeed;
//
///* Try to Get baudrate*/
//if ((ispeed = cfgetispeed(&uart_config)) < 0)
//{	warnx("ERROR getting port input speed");}
//else
//{	warnx("Port input speed is %d",ispeed);}
//
//if ((ospeed = cfgetospeed(&uart_config)) < 0)
//{	warnx("ERROR getting port output speed");}
//else
//{	warnx("Port output speed is %d",ospeed);}
//
///* Try to Set baud rate */
//ispeed = B921600;
//ospeed = B921600;
//
//if ((cfsetispeed(&uart_config,ispeed)) < 0)
//{	warnx("ERROR getting port input speed");}
//else
//{	warnx("Port input speed is now %d",ispeed);}
//
//if ((cfsetospeed(&uart_config,ospeed)) < 0)
//{	warnx("ERROR getting port output speed");}
//else
//{	warnx("Port output speed is now %d",ospeed);}


/* Clear ONLCR flag (which appends a CR for every LF) */
uart_config.c_oflag &= ~ONLCR;

/* Set the attributes */
if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0) {
	warnx("ERROR setting termios config for %s (tcsetattr)\n", uart_name);
	close(uart);
	return -1;
}

return uart;
}
